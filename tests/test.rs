#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use bare_test::{
        fdt_parser::PciSpace,
        globals::{global_val, PlatformInfoKind},
        mem::iomap,
        println,
    };
    use log::{info,debug};
    use pcie::*;
    //use igb::*;
    use project_name::Igb;


    #[test]
    fn it_works() {
        info!("This is a test log message.");
        let a = 2;
        let b = 2;
        assert_eq!(a + b, 4);
        let bar = get_igb_bar();
        let mut igb = Igb::new(bar);
        igb.init();
        //println!("test passed! {:#x}", bar);
    }

    fn get_igb_bar() ->  core::ptr::NonNull<u8> {
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt = fdt.get();

        let pcie = fdt
            .find_compatible(&["pci-host-ecam-generic"])
            .next()
            .unwrap()
            .into_pci()
            .unwrap();

            //从fdt 找到pcie的节点
        let mut pcie_regs = alloc::vec![];
        //创建一个pcie_regs的地址动态数组

        println!("test nvme");

        println!("pcie: {}", pcie.node.name);


        for reg in pcie.node.reg().unwrap() {
            println!("pcie reg: {:#x}", reg.address);
            pcie_regs.push(iomap((reg.address as usize).into(), reg.size.unwrap()));
        }

        let mut bar_alloc = SimpleBarAllocator::default();

        for range in pcie.ranges().unwrap() {
            info!("pcie range: {:?}", range);

            match range.space {
                PciSpace::Memory32 => bar_alloc.set_mem32(range.cpu_address as _, range.size as _),
                PciSpace::Memory64 => bar_alloc.set_mem64(range.cpu_address, range.size),
                _ => {}
            }
        }

        let base_vaddr = pcie_regs[0];

        info!("Init PCIE @{:?}", base_vaddr);

        let mut root = RootComplexGeneric::new(base_vaddr);

        for elem in root.enumerate(None, Some(bar_alloc)) {
            debug!("PCI {}", elem);

            if let Header::Endpoint(ep) = elem.header {
                ep.update_command(elem.root, |cmd| {
                    cmd | CommandRegister::IO_ENABLE
                        | CommandRegister::MEMORY_ENABLE
                        | CommandRegister::BUS_MASTER_ENABLE
                });

                if ep.vendor_id == 0x8086 && ep.device_id == 0x10C9 {
                    let bar_addr;
                    let bar_size;
                    match ep.bar {
                        pcie::BarVec::Memory32(bar_vec_t) => {
                            let bar0 = bar_vec_t[0].as_ref().unwrap();
                            bar_addr = bar0.address as usize;
                            bar_size = bar0.size as usize;
                        }
                        pcie::BarVec::Memory64(bar_vec_t) => {
                            let bar0 = bar_vec_t[0].as_ref().unwrap();
                            bar_addr = bar0.address as usize;
                            bar_size = bar0.size as usize;
                        }
                        pcie::BarVec::Io(_bar_vec_t) => todo!(),
                    };

                    println!("bar0: {:#x}", bar_addr);
                    let addr = iomap(bar_addr.into(), bar_size);
                    return addr;

                }
            }
        }
        panic!("no igb addr")

    }
}
