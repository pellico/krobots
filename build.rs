use std::env;
use std::io::Result;
extern crate prost_build;
fn main() -> Result<()> {
    println!("cargo:warning=OUT_DIR {}",env::var("OUT_DIR").unwrap());
    prost_build::compile_protos(&["src/tank.proto"], &["src/"])?;
    Ok(())
}
