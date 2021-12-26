use std::env;
use std::io::Result;
extern crate prost_build;
use std::path::Path;
use std::path::PathBuf;
fn get_output_path() -> PathBuf {
    //<root or manifest path>/target/<profile>/
    let manifest_dir_string = env::var("CARGO_MANIFEST_DIR").unwrap();
    let build_type = env::var("PROFILE").unwrap();
    let path = Path::new(&manifest_dir_string).join("target").join(build_type);
    return PathBuf::from(path);
}

fn copy_res(src:&str,dest_folder:&PathBuf) {
    let mut dest_path = dest_folder.clone();
    dest_path.push(src);
    std::fs::copy(src, dest_path).expect(&format!("Unable to copy {}",src));
}

fn main() -> Result<()> {
    let output_path = get_output_path();

    copy_res("body.png",&output_path);
    copy_res("bullet.png",&output_path);
    copy_res("radar.png",&output_path);
    copy_res("smoke_fire.png",&output_path);
    copy_res("turret.png",&output_path);
    println!("cargo:rerun-if-changed=src/tank.proto");
    println!("cargo:warning=OUT_DIR {}",env::var("OUT_DIR").unwrap());
    prost_build::compile_protos(&["src/tank.proto"], &["src/"])?;
    Ok(())
}
