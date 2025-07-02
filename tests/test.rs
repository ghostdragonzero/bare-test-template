#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use bare_test::println;
    use log::info;

    #[test]
    fn it_works() {
        info!("This is a test log message.");
        
        println!("test passed!");
    }
}
