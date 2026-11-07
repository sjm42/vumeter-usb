//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use anyhow::Context;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

#[path = "src/release.rs"]
mod release;

fn main() -> anyhow::Result<()> {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = PathBuf::from(env::var_os("OUT_DIR").context("missing OUT_DIR")?);
    // Cargo supplies these separately, so prerelease/build suffixes are harmless.
    let major = env::var("CARGO_PKG_VERSION_MAJOR")?.parse()?;
    let minor = env::var("CARGO_PKG_VERSION_MINOR")?.parse()?;
    let patch = env::var("CARGO_PKG_VERSION_PATCH")?.parse()?;
    let usb_device_release_bcd = release::usb_device_release(major, minor, patch).map_err(anyhow::Error::msg)?;

    File::create(out.join("memory.x"))?.write_all(include_bytes!("memory.x"))?;
    File::create(out.join("usb_device_release.rs"))?
        .write_all(format!("const USB_DEVICE_RELEASE_BCD: u16 = 0x{usb_device_release_bcd:04x};\n").as_bytes())?;
    println!("cargo:rustc-link-search={}", out.display());

    // Rebuild if the linker script or package version changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=Cargo.toml");
    println!("cargo:rerun-if-changed=src/release.rs");

    Ok(())
}
