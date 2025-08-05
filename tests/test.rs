#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;


#[bare_test::tests]
mod tests {
        use alloc::{boxed::Box, vec::Vec};
    use bare_test::{
        GetIrqConfig,
        async_std::time,
        fdt_parser::PciSpace,
        globals::{PlatformInfoKind, global_val},
        irq::{IrqHandleResult, IrqInfo, IrqParam},
        mem::mmu::{iomap, page_size},
        platform::fdt::GetPciIrqConfig,
        println,
    };
    use core::{pin::Pin, time::Duration};
    use crab_usb::{
        endpoint::{
            direction::In,
            kind::{Bulk, Isochronous},
        },
        standard::{descriptors::EndpointType, transfer::Direction},
        *,
    };
    use futures::FutureExt;
    use log::*;
    use pcie::*;

    use super::*;
    use crab_usb::*;


    struct KernelImpl;

    impl Kernel for KernelImpl {
        fn sleep<'a>(duration: Duration) -> futures::future::BoxFuture<'a, ()> {
            time::sleep(duration).boxed()
        }

        fn page_size() -> usize {
            page_size()
        }
    }

    set_impl!(KernelImpl);

    #[test]
    fn test_all() {
        spin_on::spin_on(async {
            let (mmio_base, irq_info) = get_xhci_host();
            let mut host = USBHost::new(mmio_base);
            register_irq(irq_info.unwrap(), &mut host);
            host.init().await.unwrap();
            let devices = host.probe().await.unwrap();
            println!("scan_device");

            for mut device in devices {
                // Get device descriptor
                let desc = device.descriptor().await.unwrap();
                println!("Device: {:?}", desc);
                 if let Some(index) = desc.product_string_index() {
                    let product = device.string_descriptor(index, 0).await.unwrap();
                    info!("product: {product}");
                }
                let mut interface_desc = None;
                for config in device.configuration_descriptors() {
                    info!("config: {:?}", config.configuration_value);

                    for interface in &config.interfaces {
                        info!("interface: {:?}", interface.interface_number);
                        for alt in &interface.alt_settings {
                            info!("alternate: {alt:?}");
                            if interface_desc.is_none() {
                                interface_desc = Some(alt.clone());
                            }
                        }
                    }
                }
                let interface_desc = interface_desc.unwrap();
                let mut interface = device
                    .claim_interface(
                        interface_desc.interface_number,
                        interface_desc.interface_number,
                    )
                    .await
                    .unwrap();
                info!("set interface ok");

                for ep_desc in &interface_desc.endpoints {
                    info!("endpoint: {ep_desc:?}");

                    match (ep_desc.transfer_type, ep_desc.direction) {
                        (EndpointType::Bulk, Direction::In) => {
                            let _bulk_in = interface.endpoint::<Bulk, In>(ep_desc.address).unwrap();
                            // You can use bulk_in to transfer data
                            // let mut buff = alloc::vec![0u8; 64];
                            // while let Ok(n) = bulk_in.transfer(&mut buff).await {
                            //     let data = &buff[..n];

                            //     info!("bulk in data: {data:?}",);
                            // }
                        }
                        (EndpointType::Isochronous, Direction::In) => {
                            let _iso_in = interface
                                .endpoint::<Isochronous, In>(ep_desc.address)
                                .unwrap();
                            // You can use iso_in to transfer data
                        }

                        _ => {
                            info!("unsupported endpoint type");
                        }
                    }
                }
    
            }
        });
    }

    fn get_xhci_host() -> (core::ptr::NonNull<u8>, Option<IrqInfo>) {
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt = fdt.get();

        let pcie = fdt
            .find_compatible(&["pci-host-ecam-generic", "brcm,bcm2711-pcie"])
            .next()
            .unwrap()
            .into_pci()
            .unwrap();

            //从fdt 找到pcie的节点
        let mut pcie_regs = alloc::vec![];
        //创建一个pcie_regs的地址动态数组

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

                if ep.device_type() == DeviceType::UsbController {
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
                    let irq = pcie.child_irq_info(
                        ep.address.bus(),
                        ep.address.device(),
                        ep.address.function(),
                        ep.interrupt_pin,
                    );

                    return (addr, irq);

                }
            }
        }
        panic!("no igb addr")
    }
//使能了中断模式如果不注册中断就不能正确收发数据
    fn register_irq(irq: IrqInfo, host: &mut USBHost<Xhci>) {
        let host_ptr: *mut USBHost<Xhci> = host;
        
        for one in &irq.cfgs {
            IrqParam {
                intc: irq.irq_parent,
                cfg: one.clone(),
            }
            .register_builder({
                move |_irq| {
                    unsafe {
                        (*host_ptr).handle_irq();
                    }
                    IrqHandleResult::Handled
                }
            })
            .register();
        }
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
