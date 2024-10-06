use std::collections::VecDeque;
use std::env;
use std::process;

use ixy::memory::Packet;
use ixy::*;

const BATCH_SIZE: usize = 32;

pub fn main() {
    let mut args = env::args();
    args.next();

    let pci_addr_1 = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example forwarder <pci bus id1> <pci bus id2>");
            process::exit(1);
        }
    };

    let pci_addr_2 = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example forwarder <pci bus id1> <pci bus id2>");
            process::exit(1);
        }
    };

    let mut dev1 = ixy_init(&pci_addr_1, 1, 1, -1).unwrap();
    let mut dev2 = ixy_init(&pci_addr_2, 1, 1, 0).unwrap();
    let mut buffer: VecDeque<Packet> = VecDeque::with_capacity(BATCH_SIZE);

    loop {
        forward(&mut buffer, &mut *dev1, 0, &mut *dev2, 0);
        forward(&mut buffer, &mut *dev2, 0, &mut *dev1, 0);
    }
}

fn forward(
    buffer: &mut VecDeque<Packet>,
    rx_dev: &mut dyn IxyDevice,
    rx_queue: u16,
    tx_dev: &mut dyn IxyDevice,
    tx_queue: u16,
) {
    let num_rx = rx_dev.rx_batch(rx_queue, buffer, BATCH_SIZE);

    if num_rx > 0 {
        // touch all packets for a realistic workload
        for p in buffer.iter_mut() {
            // we change a byte of the destination MAC address to ensure
            // that all packets are put back on the link (vital for VFs)
            p[3] += 1;
        }

        tx_dev.tx_batch(tx_queue, buffer);

        // drop packets if they haven't been sent out
        buffer.drain(..);
    }
}
