use core::ptr::{self, NonNull};
use core::u32;
use crate::{IgbError, IgbResult};
use tock_registers::{
    interfaces::{Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};
use log::{info,debug};


//use pool size
const MEM_POOL: usize = 4096;
const MEM_POOL_ENTRY_SIZE: usize = 2048;

register_structs! {
    IgbRegs {
        /// Device Control Register (CTRL) - 0x00000; R/W
        (0x00000 => ctrl: ReadWrite<u32, CTRL::Register>),
        (0x00004 => _rsv1),
        /// Device Status Register (STATUS) - 0x00008; R
        (0x00008 => status: ReadOnly<u32, STATUS::Register>),
        (0x0000C => _rsv2),
        /// Extended Device Control Register (CTRL_EXT) - 0x00018; R/W
        (0x00018 => ctrl_ext: ReadWrite<u32, CTRL_EXT::Register>),
        (0x0001C => _rsv3),
        /// MDI Control Register (MDIC) - 0x00020; R/W
        (0x00020 => mdic: ReadWrite<u32, MDIC::Register>),
        (0x00024 => _rsv4),
        
        (0x01524 => eims: ReadWrite<u32>),
        (0x01528 => eimc: ReadWrite<u32>),
        (0x0152c => _rsv5),
        (0x01580 => eicr: ReadWrite<u32>),
        (0x01584 => _rsv6),
        (0x05400 => addrl:ReadOnly<u32>),
        (0x05404 => addrh:ReadOnly<u32>),
/*         /// EEPROM/Flash Control Register (EEC) - 0x00010; R/W
        (0x00100 => eec: ReadWrite<u32, EEC::Register>),
        /// EEPROM Read Register (EERD) - 0x00014; RW
        (0x00014 => eerd: ReadWrite<u32, EERD::Register>),
        /// Receive Control Register (RCTL) - 0x00100; R/W
        (0x00100 => rctl: ReadWrite<u32, RCTL::Register>),
        /// Transmit Control Register (TCTL) - 0x00400; R/W
        (0x00400 => tctl: ReadWrite<u32, TCTL::Register>),
        /// Interrupt Cause Read Register (ICR) - 0x01500; RC/W1C
        (0x01500 => icr: ReadWrite<u32, ICR::Register>),
        /// Interrupt Mask Set/Read Register (IMS) - 0x01508; R/W
        (0x01508 => ims: ReadWrite<u32, IMS::Register>),
        /// Interrupt Mask Clear Register (IMC) - 0x0150C; WO
        (0x0150C => imc: WriteOnly<u32, IMC::Register>),
*/
        (0x5408 => @END),
    }
}

register_bitfields![u32,
    CTRL [
        /// Software Reset
        RST OFFSET(3) NUMBITS(1) [],
        /// Set Link Up
        SLU OFFSET(6) NUMBITS(1) [],
        /// Speed Selection
        SPEED OFFSET(8) NUMBITS(2) [
            Speed10 = 0b00,
            Speed100 = 0b01,
            Speed1000 = 0b10
        ],
        /// Transmit Flow Control Enable
        TFCE OFFSET(10) NUMBITS(1) [],
        /// Receive Flow Control Enable
        RFCE OFFSET(11) NUMBITS(1) [],
        /// PHY Reset
        DEV_RST OFFSET(26) NUMBITS(1) [],
        /// PHY Reset
        PHY_RST OFFSET(31) NUMBITS(1) []
    ],
    STATUS [
        /// Link Up
        LU OFFSET(1) NUMBITS(1) [],
        /// Full Duplex
        FD OFFSET(2) NUMBITS(1) [],
        /// Speed Indication
        SPEED OFFSET(6) NUMBITS(2) [
            Speed10 = 0b00,
            Speed100 = 0b01,
            Speed1000 = 0b10,
        ],
        /// Link Status Change
        LSC OFFSET(22) NUMBITS(1) [],
    ],
    CTRL_EXT [
        /// EEPROM Reset
        EE_RST OFFSET(8) NUMBITS(1) [],
        /// Link Mode Selection
        LINK_MODE OFFSET(12) NUMBITS(2) [
            InternalPHY = 0b00,
            SGMII = 0b10,
            SerDes = 0b11
        ],
    ],
    MDIC [
        /// PHY Address
        PHYADD OFFSET(0) NUMBITS(5) [],
        /// Register Address
        REGADD OFFSET(5) NUMBITS(5) [],
        /// Opcode
        OPCODE OFFSET(26) NUMBITS(2) [
            Read = 0b10,
            Write = 0b01
        ],
        /// Ready
        RDY OFFSET(28) NUMBITS(1) [],
        /// Data
        DATA OFFSET(18) NUMBITS(16) [],
        ERROR OFFSET(30) NUMBITS(1) [],
    ],
    EEC [
        /// EEPROM Clock
        EESK OFFSET(0) NUMBITS(1) [],
        /// EEPROM Data In
        EEDI OFFSET(1) NUMBITS(1) [],
        /// EEPROM Chip Select
        EECS OFFSET(2) NUMBITS(1) [],
        /// EEPROM Data Out
        EEDO OFFSET(3) NUMBITS(1) [],
        /// Flash Write Enable
        FWE OFFSET(4) NUMBITS(2) [],
    ],
    /* 
    EERD [
        /// Address
        ADDR OFFSET(0) NUMBITS(16) [],
        /// Data
        DATA OFFSET(16) NUMBITS(16) [],
        /// Start Read
        START OFFSET(31) NUMBITS(1) [],
        /// Done
        DONE OFFSET(30) NUMBITS(1) [],
    ],
    RCTL [
        /// Receive Enable
        EN OFFSET(1) NUMBITS(1) [],
        /// Store Bad Packets
        SBP OFFSET(2) NUMBITS(1) [],
        /// Unicast Promiscuous
        UPE OFFSET(3) NUMBITS(1) [],
        /// Multicast Promiscuous
        MPE OFFSET(4) NUMBITS(1) [],
        /// Broadcast Enable
        BAM OFFSET(15) NUMBITS(1) [],
        /// Maximum Frame Length
        MAXFL OFFSET(16) NUMBITS(10) [],
    ],
    TCTL [
        /// Transmit Enable
        EN OFFSET(1) NUMBITS(1) [],
        /// Pad Short Packets
        PSP OFFSET(3) NUMBITS(1) [],
        /// Collision Threshold
        CT OFFSET(4) NUMBITS(4) [],
        /// Collision Distance
        COLD OFFSET(8) NUMBITS(12) [],
    ],
    ICR [
        /// Link Status Change
        LSC OFFSET(2) NUMBITS(1) [],
        /// Receive Descriptor Minimum Threshold
        RXDMT0 OFFSET(4) NUMBITS(1) [],
        /// Transmit Descriptor Empty
        TXDE0 OFFSET(12) NUMBITS(1) [],
        /// Watchdog Timeout
        WDT OFFSET(26) NUMBITS(1) [],
    ],
    IMS [
        /// Link Status Change Mask
        LSC OFFSET(2) NUMBITS(1) [],
        /// Receive Descriptor Minimum Threshold Mask
        RXDMT0 OFFSET(4) NUMBITS(1) [],
        /// Transmit Descriptor Empty Mask
        TXDE0 OFFSET(12) NUMBITS(1) [],
        /// Watchdog Timeout Mask
        WDT OFFSET(26) NUMBITS(1) [],
    ],
    */
    IMC [
        /// Link Status Change Mask Clear
        LSC OFFSET(2) NUMBITS(1) [],
        /// Receive Descriptor Minimum Threshold Mask Clear
        RXDMT0 OFFSET(4) NUMBITS(1) [],
        /// Transmit Descriptor Empty Mask Clear
        TXDE0 OFFSET(12) NUMBITS(1) [],
        /// Watchdog Timeout Mask Clear
        WDT OFFSET(26) NUMBITS(1) [],
    ]
];

pub struct Igb {
    addr:NonNull<IgbRegs>,
}

impl Igb {
    pub fn new(bar0: NonNull<u8>) -> Self {
        info!("bar0 = {:?}", bar0);
        let bra0 = unsafe {
            bar0.as_ptr().cast::<IgbRegs>()
        };

        Self {
            addr:NonNull::new(bra0).expect("无效地址"),
        }
    }
    fn regs_mut(&mut self) -> &mut IgbRegs{
        unsafe {self.addr.as_mut()}
    }

    fn regs(& self) -> & IgbRegs{
        unsafe {self.addr.as_ref()}
    }

    fn disable_interrupts(&mut self){
        self.regs_mut().eims.set(0);
        //关闭mask
        self.regs_mut().eimc.set(u32::MAX);
        self.regs_mut().eicr.get();
    }

    pub fn init(&mut self){
        self.disable_interrupts();
        self.regs_mut().ctrl.write(CTRL::DEV_RST::SET);
        while (self.regs().ctrl.read(CTRL::DEV_RST) == 1){

        }
        //do reset 
        info!("reset success");
        // self.set_flags32(IGB_CTRL, IGB_CTRL_LNK_RST);
        info!("link mode is defaut 00");
        self.disable_interrupts();
        //self.set_flags32(IGB_CTRL_EXT, IGB_CTRL_EXT_DRV_LOAD);
        
        let mac = self.get_mac_addr();

        info!(
            "mac address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        );

        info!("start link");
        //self.set_speed_focred();
        self.set_up_link_up();
        

        //info!("set SLU ok");
        loop{
            let status = self.regs().status.read(STATUS::LU);
            //info!("status {:b}", status);
            if (status == 1){
                break;
            }
        }
        info!("link up ok");
        //self.wait_set_reg32(IGB_EEC, IXGBE_EEC_ARD);
        //igb可能不需要
        //RO 位置 应该只是利用这个函数等待为1 说明读取完毕
        /* 
        //self.setup_rx_mode();
        //self.setup_tctl();
        self.setup_rctl();
        self.init_rx();
        self.init_tx();
        */

    }
    
    

    fn get_mac_addr(&self) -> [u8; 6] {
        let low = self.regs().addrl.get();
        let high = self.regs().addrh.get();

        [
            (low & 0xff) as u8,
            (low >> 8 & 0xff) as u8,
            (low >> 16 & 0xff) as u8,
            (low >> 24) as u8,
            (high & 0xff) as u8,
            (high >> 8 & 0xff) as u8,
        ]
    }
    /* 
    fn set_speed_focred(&self) {
        let speed = self.get_reg32(IGB_STATUS);
        info!("speed = {:b}", (speed >> 6 & 0x3) as u8);

        self.set_flags32(IGB_CTRL, IGB_CTRL_FRCSPD);
        let mut reg = self.get_reg32(IGB_CTRL);
        reg &= !(1<<9);
        reg |= 1<<8;
        self.set_reg32(IGB_CTRL, reg);

        let speed = self.get_reg32(IGB_STATUS);
        info!("new speed = {:b}", (speed >> 6 & 0x3) as u8);
    }
    */

    fn set_up_link_up(&mut self) {
        let mut mii_reg = self.phy_read(0);
        mii_reg &= 0xffff;
        mii_reg &= !(1<<11);
        info!("power up write_mii{:b}", mii_reg);
        //let status = self.get_reg32(IGB_STATUS);
        //info!("reset end status {:b}", status);
        self.phy_write(0, mii_reg);

        mii_reg = self.phy_read(0);
        mii_reg &= 0xffff;
        mii_reg |= 1<<9;
        info!("rs_atu write_mii{:b}", mii_reg);
        //let status = self.get_reg32(IGB_STATUS);
        //info!("reset end status {:b}", status);
        self.phy_write(0, mii_reg);



        self.phy_write(0, mii_reg);
        mii_reg = self.phy_read(0);
        mii_reg &= 0xffff;
        mii_reg |= 1<<12;
        info!("en_atu write_mii{:b}", mii_reg);
        //let status = self.get_reg32(IGB_STATUS);
        //info!("reset end status {:b}", status);
        self.phy_write(0, mii_reg);


        //self.set_flags32(IGB_CTRL, IGB_CTRL_START);
        //FRCSPD defaut is 0 FRCDPLX

    }


/* 

    fn get_reg32(&self, reg: u32) -> u32 {
        //assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe { ptr::read_volatile((self.addr as usize + reg as usize) as *mut u32) }
    }

    /// Sets the register at `self.addr` + `reg` to `value`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn set_reg32(&self, reg: u32, value: u32) {
        //assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe {
            ptr::write_volatile((self.addr as usize + reg as usize) as *mut u32, value);
        }
    }

    /// Sets the `flags` at `self.addr` + `reg`.
    fn set_flags32(&self, reg: u32, flags: u32) {
        self.set_reg32(reg, self.get_reg32(reg) | flags);
    }

    /// Clears the `flags` at `self.addr` + `reg`.
    fn clear_flags32(&self, reg: u32, flags: u32) {
        self.set_reg32(reg, self.get_reg32(reg) & !flags);
    }

    /// Waits for `self.addr` + `reg` to clear `value`.
    fn wait_clear_reg32(&self, reg: u32, value: u32) {
        loop {
            let current = self.get_reg32(reg);
            if (current & value) == 0 {
                break;
            }
            // `thread::sleep(Duration::from_millis(100));`
            // let _ = H::wait_ms(100);
            //let _ = H::wait_until(Duration::from_millis(100));
        }
    }

    /// Waits for `self.addr` + `reg` to set `value`.
    fn wait_set_reg32(&self, reg: u32, value: u32) {
        loop {
            let current = self.get_reg32(reg);
            if (current & value) == value {
                break;
            }
            //let _ = H::wait_until(Duration::from_millis(100));
        }
    }
    */

    fn phy_read(&mut self, offset: u32) -> u16{
        let  mdic_cmd = (offset << 16) | (1 << 21) | (1<< 27);
        info!("phy read cmd {:b}", mdic_cmd);
        self.regs_mut().mdic.set(mdic_cmd);

        loop {
            let mdic_cmd = self.regs().mdic.extract();
            if mdic_cmd.is_set(MDIC::RDY){
                return mdic_cmd.read(MDIC::DATA) as u16;
            }
            if mdic_cmd.is_set(MDIC::ERROR){
                info!("read error");
                return 0;
            }
        }
    }

    fn phy_write(&mut self, offset: u32, data:u16) -> bool{
        let mdic_cmd = (offset << 16) | (1 << 21) | (data as u32) | (1<<26);
        info!("phy write cmd {:b}", mdic_cmd);
        self.regs_mut().mdic.set(mdic_cmd);

        loop {
            let mdic_cmd = self.regs().mdic.extract();
            if mdic_cmd.is_set(MDIC::RDY){
                return true;
            }
            if mdic_cmd.is_set(MDIC::ERROR){
                info!("read error");
                return false;
            }
            
        }
    }
    

}