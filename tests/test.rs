#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;


#[bare_test::tests]
mod tests {
        use alloc::{boxed::Box, vec::Vec};
    use bare_test::{
        async_std::time, fdt_parser::PciSpace, globals::{global_val, PlatformInfoKind}, irq::{IrqHandleResult, IrqInfo, IrqParam}, mem::mmu::{iomap, page_size}, platform::fdt::GetPciIrqConfig, platform_if::RegionKind, println, GetIrqConfig
    };
    use dma_api::DSliceMut;
    use core::{pin::Pin, time::Duration};
    use crab_usb::{
        endpoint::{
            direction::In,
            kind::{Bulk, Interrupt, Isochronous},
        }, standard::transfer::control::ControlType, *
    };
    use futures::FutureExt;
    use log::*;
    use pcie::*;
    use crab_usb::standard::transfer::control::Control;
    use crab_usb::standard::transfer::control::Recipient;



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
                // 获取设备描述符
                let desc = device.descriptor().await.unwrap();
                println!("Device: {:?}", desc);

                if let Some(index) = desc.product_string_index {
                    let product = device.string_descriptor(index, 0).await.unwrap();
                    println!("Product: {}", product);
                }

                // 1. 查找UVC VideoStreaming接口和带Isochronous In端点的Alt Setting
                let mut vs_alt_setting = None;
                let mut vs_interface_number = 0;
                for config in device.configuration_descriptors() {
                    for interface in &config.interfaces {
                        for alt in &interface.alt_settings {
                            // UVC VideoStreaming接口: bInterfaceClass=0x0E, bInterfaceSubClass=0x02
                            let has_iso_in_ep = alt.endpoints.len();
                            if has_iso_in_ep == 1 {
                                println!("have ep ");
                                vs_alt_setting = Some(alt.clone());
                                vs_interface_number = alt.interface_number;
                                break;
                            }
                        }

                    }
                }
                let vs_alt_setting = vs_alt_setting.expect("No UVC VideoStreaming interface found");
                println!("vs_interface_number {}", vs_interface_number);



                // 构造UVC VS_PROBE_CONTROL的Control参数
                let probe_param = Control {
                    transfer_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: standard::transfer::control::Request::Other(0x01), // SET_CUR
                    value: 0x0100, // VS_PROBE_CONTROL (0x01 << 8)
                    index: vs_interface_number as u16,
                };
                let probe_get_param = Control {
                    transfer_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: crab_usb::standard::transfer::control::Request::Other(0x81), // GET_CUR
                    value: 0x0100, // VS_PROBE_CONTROL
                    index: vs_interface_number as u16,
                };
                let commit_param = Control {
                    transfer_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: crab_usb::standard::transfer::control::Request::Other(0x01), // SET_CUR
                    value: 0x0200, // VS_COMMIT_CONTROL (0x02 << 8)
                    index: vs_interface_number as u16,
                };
                let commit_get_param = Control {
                    transfer_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: crab_usb::standard::transfer::control::Request::Other(0x81), // GET_CUR
                    value: 0x0200, // VS_COMMIT_CONTROL
                    index: vs_interface_number as u16,
                };
                let current_probe_get_param = Control {
                    transfer_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: crab_usb::standard::transfer::control::Request::Other(0x81), // GET_CUR
                    value: 0x0100, // VS_PROBE_CONTROL
                    index: vs_interface_number as u16,
                };

// 2. 读取当前Probe配置
let mut probe_data = alloc::vec![0u8; 26];
device.control_in(current_probe_get_param, &mut probe_data).await.unwrap();

// 3. 解析关键字段（小端序）
let format_index = probe_data[2];  // bFormatIndex 位置
let frame_index = probe_data[3];   // bFrameIndex 位置
let frame_interval = u32::from_le_bytes([
    probe_data[4], probe_data[5], 
    probe_data[6], probe_data[7]  // dwFrameInterval
]);

log::info!(
    "Current Format: index={}, Frame index={}, Interval={}ns",
    format_index, frame_index, frame_interval * 100  // 单位100ns
);

// 4. 修改为想要的格式配置 ----------------------------------
// 示例：设置为MJPEG格式(index=1), 1280x720分辨率(index=3), 30fps
probe_data[2] = 1;  // bFormatIndex = MJPEG
probe_data[3] = 1;  // bFrameIndex = 1280x720

// 设置帧间隔 (30fps = 1/30 ≈ 333333 * 100ns)
let new_interval:i32 = 333_333;
probe_data[4..8].copy_from_slice(&new_interval.to_le_bytes());

// 5. 提交新Probe参数
device.control_out(probe_param, &probe_data).await.unwrap();

// 6. 获取设备调整后的实际参数
let mut adjusted_data = alloc::vec![0u8; 26];
device.control_in(probe_get_param, &mut adjusted_data).await.unwrap();

// 7. 提交最终配置
device.control_out(commit_param, &adjusted_data).await.unwrap();

log::info!("Video format updated successfully!");

// 2. 发送STREAMING_ON命令
let streaming_on_param = Control {
    transfer_type: ControlType::Class,
    recipient: Recipient::Interface,
    request: crab_usb::standard::transfer::control::Request::Other(0x01), // SET_CUR
    value: 0x0200, // VS_COMMIT_CONTROL (0x02 << 8)
    index: vs_interface_number as u16,
};
device.control_out(streaming_on_param, &[]).await.unwrap();

                println!("setcurOK");
                // 3. 切换到带Isochronous端点的Alt Setting
                let mut interface = device
                    .claim_interface(
                        vs_interface_number,
                        vs_alt_setting.alternate_setting,
                    )
                    .await
                    .unwrap();
                info!("set interface ok");

                // 4. 读取等时端点数据
                // 自动查找Isochronous In端点号
                //let mut iso_in_ep_addr = None;
                for ep in &vs_alt_setting.endpoints {
                    println!("ep: {:?}", ep);
                    
                }
                

                let mut ep = interface.endpoint::<Isochronous, In>(129).unwrap();
                // let mut assembler = FrameAssembler::<{300 * 1024}>::new();
                loop {
                    let mut buf =  alloc::vec![0u8; 64 * 1024];
                    let len = ep.transfer(&mut buf).unwrap().await.unwrap_or(0);
                    if len > 0 {
                        println!("Got video data len = {}", len);

                    }
                }
            }
        });
    }

/* 
pub struct FrameAssembler<const MAX_FRAME_SIZE: usize> {
    current_frame: Vec<u8, MAX_FRAME_SIZE>,
    in_frame: bool,
    frame_count: u32,
}

impl<const MAX_FRAME_SIZE: usize> FrameAssembler<MAX_FRAME_SIZE> {
    /// 创建新的帧重组器
    pub fn new() -> Self {
        Self {
            current_frame: Vec::new(),
            in_frame: false,
            frame_count: 0,
        }
    }
    
    /// 处理接收到的数据包
    pub fn process_packet(&mut self, data: &[u8]) -> Option<&[u8]> {
        if data.len() < 12 {
            return None; // 无效数据包
        }
        
        let header_length = data[0] as usize;
        let header_info = data[1];
        
        // 确保有足够的长度
        if header_length > data.len() {
            return None;
        }
        
        let payload = &data[header_length..];
        
        // 检查帧开始标志 (bit 2: start-of-frame)
        if header_info & 0x04 != 0 {
            self.current_frame.clear();
            self.in_frame = true;
        }
        
        // 如果正在帧中，添加有效载荷
        if self.in_frame {
            // 检查是否有足够空间
            if self.current_frame.capacity() - self.current_frame.len() < payload.len() {
                // 缓冲区不足 - 重置状态
                self.in_frame = false;
                self.current_frame.clear();
                return None;
            }
            
            // 添加数据到当前帧
            if self.current_frame.extend_from_slice(payload).is_err() {
                // 扩展失败 - 重置状态
                self.in_frame = false;
                self.current_frame.clear();
                return None;
            }
        }
        
        // 检查帧结束标志 (bit 1: end-of-frame)
        if header_info & 0x02 != 0 && self.in_frame {
            self.in_frame = false;
            self.frame_count += 1;
            
            // 返回当前帧的引用
            return Some(&self.current_frame);
        }
        
        None
    }
    
    /// 获取当前帧计数
    pub fn frame_count(&self) -> u32 {
        self.frame_count
    }
    
    /// 重置帧重组器
    pub fn reset(&mut self) {
        self.current_frame.clear();
        self.in_frame = false;
        self.frame_count = 0;
    }
    }
*/
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
}