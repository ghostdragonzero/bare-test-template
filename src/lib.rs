#![no_std]

extern crate alloc;
mod igb;

pub use igb::*;


#[derive(Debug)]
/// Error type for Ixgbe functions.
pub enum IgbError {
    /// Queue size is not aligned.
    QueueNotAligned,
    /// Threr are not enough descriptors available in the queue, try again later.
    QueueFull,
    /// No memory
    NoMemory,
    /// Allocated page not aligned.
    PageNotAligned,
    /// The device is not ready.
    NotReady,
    /// Invalid `queue_id`.
    InvalidQueue,
}

/// Result type for Ixgbe functions.
pub type IgbResult<T = ()> = Result<T, IgbError>;