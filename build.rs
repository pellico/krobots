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
    
    for entry in std::fs::read_dir(src).unwrap() {
        match entry {
            Ok(src) => {
                let mut dest_path = dest_folder.clone();
                dest_path.push(src.file_name());
                std::fs::copy(src.path(), dest_path).expect(&format!("Unable to copy something"));
            }
            Err(_) => {println!("cargo:warning=Error copying resources")}
        }
        
    }
    
    
}


fn main() -> Result<()> {
    let output_path = get_output_path();
    println!("cargo:rerun-if-changed=resources");
    println!("cargo:rerun-if-changed=src/tank.proto");
    copy_res("resources",&output_path);
    println!("cargo:warning=OUT_DIR {}",env::var("OUT_DIR").unwrap());
    prost_build::compile_protos(&["src/tank.proto"], &["src/"])?;
    Ok(())
}
