//! Encoding shared by the build script and its host-side regression tests.

pub const fn usb_device_release(major: u16, minor: u16, patch: u16) -> Result<u16, &'static str> {
    if major > 99 || minor > 9 || patch > 9 {
        return Err("USB release MMmp requires major <= 99, minor <= 9 and patch <= 9");
    }
    Ok(((major / 10) << 12) | ((major % 10) << 8) | (minor << 4) | patch)
}
