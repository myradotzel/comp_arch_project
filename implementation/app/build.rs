use std::process::Command;
use std::env;
use std::path::Path;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
/* */
    // Hard-coded for now
    Command::new("msp430-elf-gcc").args(&["src/runtime.c", "-c", "-msmall", "-mcpu=msp430","-mmcu=msp430fr5994", "-g", "-o"])
                       .arg(&format!("{}/intermittent.o", out_dir))
                       .status().unwrap();
    Command::new("msp430-elf-ar").args(&["crus", "libintermittent.a", "intermittent.o"])
                      .current_dir(&Path::new(&out_dir))
                      .status().unwrap();

    println!("cargo:rustc-link-search=native={}", out_dir);
    println!("cargo:rustc-link-lib=static=intermittent");
    println!("cargo:rerun-if-changed=src/runtime.c");
}