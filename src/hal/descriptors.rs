use volatile::Volatile;
use bit_field::BitField;

pub struct PhysicalAddress{
    pub value: usize
}

impl PhysicalAddress {
    fn value(&self) -> usize {
        self.value
    }
}

pub trait Descriptor {
    /// set all fields to 0
    fn clear(&mut self);
}

// Transmit descriptor bits
/// Tx Command: End of Packet
pub const TX_CMD_EOP:                      u8 = 1 << 0;     
/// Tx Command: Insert MAC FCS
pub const TX_CMD_IFCS:                     u8 = 1 << 1;     
/// Tx Command: Insert Checksum
pub const TX_CMD_IC:                       u8 = 1 << 2;     
/// Tx Command: Report Status
pub const TX_CMD_RS:                       u8 = 1 << 3;     
/// Tx Command: Descriptor Extension (Advanced format)
pub const TX_CMD_DEXT:                     u8 = 1 << 5;  
/// Tx Command: VLAN Packet Enable
pub const TX_CMD_VLE:                      u8 = 1 << 6;     
/// Tx Command: Interrupt Delay Enable
pub const TX_CMD_IDE:                      u8 = 1 << 7;     
/// Tx Status: descriptor Done
pub const TX_STATUS_DD:                    u8 = 1 << 0;
/// Tx Descriptor Type: advanced
pub const TX_DTYP_ADV:                     u8 = 0x3 << 4;
/// Tx Descriptor paylen shift
/// The paylen is located at bit 46 in the upper 64 bits of the advanced Tx descriptor.
/// Since we have divided the upper 64 bits into 4 parts (u16,u8,u8,u32),
/// the paylen is then located at bit 14 of the upper 32 bits of the descriptor.
pub const TX_PAYLEN_SHIFT:                 u8 = 46 - 32; //(actual offset - offset of variable) 


// Receive descriptor bits 
/// Rx Status: Descriptor Done
pub const RX_STATUS_DD:                    u8 = 1 << 0;
/// Rx Status: End of Packet
pub const RX_STATUS_EOP:                   u8 = 1 << 1;

/// This struct is a Legacy Transmit Descriptor. 
/// It's the descriptor type used in older Intel NICs and the E1000 driver.
#[repr(C)]
pub struct LegacyTxDescriptor {
    /// The starting physical address of the transmit buffer
    pub phys_addr:  Volatile<u64>,
    pub other: Volatile<u64>
}

impl LegacyTxDescriptor {
    fn init(&mut self) {
        self.phys_addr.write(0);
        self.other.write(0);
    }

    #[inline(always)]
    pub fn send(&mut self, transmit_buffer_addr: PhysicalAddress, transmit_buffer_length: u16, rs_bit: u8) {
        self.phys_addr.write(transmit_buffer_addr.value() as u64);
        self.other.write(transmit_buffer_length as u64 | (((TX_CMD_EOP | TX_CMD_IFCS | rs_bit) as u64) << 24));
    }

    #[inline(always)]
    pub fn desc_done(&self) -> bool {
        (self.other.read() & (1 << 32)) == (1 << 32)
    }
}

impl Descriptor for LegacyTxDescriptor {
    /// Set all fields to 0
    fn clear(&mut self) {
        self.phys_addr.write(0);
        self.other.write(0);
    }
}

/// Advanced Transmit Descriptor used by the `ixgbe` NIC driver.
///
/// # Two usage modes
/// It has 2 modes: Read and Write Back, both of which use the whole 128 bits. 
/// There is one transmit descriptor per transmit buffer; it can be converted between these 2 modes.
///
/// Read contains the addresses that the driver writes.
/// Write Back contains information the hardware writes on receiving a packet.
///
/// More information can be found in the 82599 datasheet.

#[repr(C)]
pub struct AdvancedTxDescriptor {
    /// Starting physical address of the receive buffer for the packet.
    pub packet_buffer_address:  Volatile<u64>,
    pub other:  Volatile<u64>,

//     /// Length of data buffer
//     pub data_len: Volatile<u16>,
//     /// A multi-part field:
//     /// * `dtyp`: Descriptor Type, occupies bits `[7:4]`,
//     /// * `mac`: options to apply LinkSec and time stamp, occupies bits `[3:2]`.
//     pub dtyp_mac_rsv : Volatile<u8>,
//     /// Command bits
//     pub dcmd:  Volatile<u8>,
//     /// A multi-part field:
//     /// * `paylen`: the size in bytes of the data buffer in host memory.
//     ///   not including the fields that the hardware adds), occupies bits `[31:14]`.
//     /// * `popts`: options to offload checksum calculation, occupies bits `[13:8]`.
//     /// * `sta`: status of the descriptor (whether it's in use or not), occupies bits `[3:0]`.
//     pub paylen_popts_cc_idx_sta: Volatile<u32>,
}

impl AdvancedTxDescriptor {
    #[inline(always)]
    pub(crate) fn send(&mut self, transmit_buffer_addr: PhysicalAddress, transmit_buffer_length: u16, rs_bit: u8) {
        self.packet_buffer_address.write(transmit_buffer_addr.value() as u64);
        self.other.write(
            (((transmit_buffer_length as u64) << TX_PAYLEN_SHIFT) << 32) | 
            (((TX_CMD_DEXT | TX_CMD_IFCS | TX_CMD_EOP | rs_bit) as u64) << 24) |
            ((TX_DTYP_ADV as u64) << 16) | 
            transmit_buffer_length as u64
        );

        // self.data_len.write(transmit_buffer_length);
        // self.dtyp_mac_rsv.write(TX_DTYP_ADV);
        // self.paylen_popts_cc_idx_sta.write((transmit_buffer_length as u32) << TX_PAYLEN_SHIFT);
        // self.dcmd.write(TX_CMD_DEXT | TX_CMD_IFCS | TX_CMD_EOP | rs_bit); // ToDo:: REmove RS field because we've set the thresholds
    }

    // #[inline(always)]
    // pub fn wait_for_packet_tx(&self) {
    //     while (self.paylen_popts_cc_idx_sta.read() as u8 & TX_STATUS_DD) == 0 {
    //         // error!("tx desc status: {:#X}", self.desc.read());
    //     } 
    // }

    // #[inline(always)]
    // pub fn desc_done(&self) -> bool {
    //     (self.paylen_popts_cc_idx_sta.read() as u8 & TX_STATUS_DD) == TX_STATUS_DD
    // }

}

impl Descriptor for AdvancedTxDescriptor {
    /// Set all fields to 0
    fn clear(&mut self) {
        self.packet_buffer_address.write(0);
        self.other.write(0);
        // self.paylen_popts_cc_idx_sta.write(0);
        // self.dcmd.write(0);
        // self.dtyp_mac_rsv.write(0);
        // self.data_len.write(0);
    }
}



/// This struct is a Legacy Receive Descriptor. 
/// The driver writes to the upper 64 bits, and the NIC writes to the lower 64 bits.
/// It's the descriptor type used in older Intel NICs and the E1000 driver.

#[repr(C)]
pub struct LegacyRxDescriptor {
    /// The starting physical address of the receive buffer
    pub phys_addr:  Volatile<u64>,      
    pub other: Volatile<u64>,
    // /// Length of the receive buffer in bytes
    // pub length:     ReadOnly<u16>,
    // /// Checksum value of the packet after the IP header till the end 
    // pub checksum:   ReadOnly<u16>,
    // /// Status bits which tell if the descriptor has been used
    // pub status:     Volatile<u8>,
    // /// Receive errors
    // pub errors:     ReadOnly<u8>,
    // /// Vlan tags
    // pub vlan:       ReadOnly<u16>,
}

impl LegacyRxDescriptor {
    pub fn init(&mut self, packet_buffer_address: PhysicalAddress) {
        self.phys_addr.write(packet_buffer_address.value() as u64);
        self.other.write(0);
    }

    #[inline(always)]
    pub fn set_packet_address(&mut self, packet_buffer_address: PhysicalAddress) {
        self.phys_addr.write(packet_buffer_address.value() as u64);
        self.other.write(0);
    }

    /// Returns (descriptor done bit, packet length)
    #[inline(always)]
    pub fn rx_metadata(&self) -> (bool, u16) {
        let metadata = self.other.read();
        ((metadata & (1 << 32)) == (1 << 32), metadata as u16 & 0xFFFF)
    }

    pub fn reset_status(&mut self) {
        self.other.write(0);
    }

    // pub fn descriptor_done(&self) -> bool {
    //     (self.status.read() & RX_STATUS_DD) == RX_STATUS_DD
    // }

    // pub fn end_of_packet(&self) -> bool {
    //     (self.status.read() & RX_STATUS_EOP) == RX_STATUS_EOP        
    // }

    // pub fn length(&self) -> u64 {
    //     self.length.read() as u64
    // }
}

impl Descriptor for LegacyRxDescriptor {
    fn clear(&mut self) {
        self.phys_addr.write(0);
        self.other.write(0);
    }
}

// use core::fmt;
// impl fmt::Debug for LegacyRxDescriptor {
//     fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//             write!(f, "{{addr: {:#X}, length: {}, checksum: {}, status: {}, errors: {}, special: {}}}",
//                     self.phys_addr.read(), self.length.read(), self.checksum.read(), self.status.read(), self.errors.read(), self.vlan.read())
//     }
// }
/// Advanced Receive Descriptor used in the Ixgbe driver.
/// It has 2 modes: Read and Write Back, both of which use the whole 128 bits. 
/// There is one receive descriptor per receive buffer that can be converted between these 2 modes.
/// Read contains the addresses that the driver writes.
/// Write Back contains information the hardware writes on receiving a packet.
/// More information can be found in the 82599 datasheet.

#[repr(C)]
pub struct AdvancedRxDescriptor {
    /// Starting physcal address of the receive buffer for the packet.
    pub packet_buffer_address:  Volatile<u64>,
    /// Starting physcal address of the receive buffer for the header.
    /// This field will only be used if header splitting is enabled. 
    pub header_buffer_address:  Volatile<u64>,
}



impl AdvancedRxDescriptor {
    #[inline(always)]
    pub(crate) fn set_packet_address(&mut self, packet_buffer_address: PhysicalAddress) {
        self.packet_buffer_address.write(packet_buffer_address.value() as u64);
        self.header_buffer_address.write(0);
    }

    /// Returns (descriptor done bit, packet length)
    #[inline(always)]
    pub fn rx_metadata(&self) -> (bool, u16) {
        let metadata = self.header_buffer_address.read();
        ((metadata & RX_STATUS_DD as u64) == RX_STATUS_DD as u64, (metadata >> 32) as u16 & 0xFFFF)
    }

    #[inline(always)]
    pub fn descriptor_done(&self) -> bool{
        (self.get_ext_status() & RX_STATUS_DD as u64) == RX_STATUS_DD as u64
    }

    #[inline(always)]
    pub fn end_of_packet(&self) -> bool {
        (self.get_ext_status() & RX_STATUS_EOP as u64) == RX_STATUS_EOP as u64        
    }

    #[inline(always)]
    pub fn length(&self) -> u64 {
        self.get_pkt_len() as u64
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Status information indicates whether a descriptor has been used 
    /// and whether the buffer is the last one for a packet
    #[inline(always)]
    pub fn get_ext_status(&self) -> u64{
        self.header_buffer_address.read() & 0xFFFFF //.get_bits(0..19) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the number of bytes posted to the packet buffer
    #[inline(always)]
    pub fn get_pkt_len(&self) -> u64{
        (self.header_buffer_address.read() >> 32) & 0xFFFF //.get_bits(32..47) 
    }
}


// functions that don't have to be verified
impl AdvancedRxDescriptor {
    pub(crate) fn init (&mut self, packet_buffer_address: PhysicalAddress) {
        self.packet_buffer_address.write(packet_buffer_address.value() as u64);
        // set the header address to 0 because packet splitting is not supposed to be enabled in the 82599
        self.header_buffer_address.write(0);
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the packet type that was used for the Receive Side Scaling hash function.
    pub fn get_rss_type(&self) -> u64{
        self.packet_buffer_address.read().get_bits(0..3) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the packet type as identified by the hardware.
    pub fn get_packet_type(&self) -> u64{
        self.packet_buffer_address.read().get_bits(4..16) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the number of Receive Side Coalesced packets that start in this descriptor.
    pub fn get_rsccnt(&self) -> u64{
        self.packet_buffer_address.read().get_bits(17..20) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the size of the packet header in bytes.
    pub fn get_hdr_len(&self) -> u64{
        self.packet_buffer_address.read().get_bits(21..30) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// When set to 1b, indicates that the hardware has found the length of the header.
    pub fn get_sph(&self) -> bool{
        self.packet_buffer_address.read().get_bit(31) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the Receive Side Scaling hash.
    pub fn get_rss_hash(&self) -> u64{
        self.packet_buffer_address.read().get_bits(32..63) 
    }

    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns the Flow Director Filter ID if the packet matches a filter.
    pub fn get_fdf_id(&self) -> u64{
        self.packet_buffer_address.read().get_bits(32..63) 
    }
    
    /// Write Back mode function for the Advanced Receive Descriptor.
    /// Returns errors reported by hardware for different packet types
    pub fn get_ext_error(&self) -> u64{
        self.header_buffer_address.read().get_bits(20..31) 
    }
    
    /// Write Back mode function for the Advanced Receive Descriptor.
    /// If the vlan header is stripped from the packet, then the 16 bits of the VLAN tag are posted here
    pub fn get_vlan_tag(&self) -> u64{
        self.header_buffer_address.read().get_bits(48..63) 
    }    
}

impl Descriptor for AdvancedRxDescriptor {
    fn clear(&mut self) {
        self.packet_buffer_address.write(0);
        self.header_buffer_address.write(0);
    }
}