use std::env;
use std::fs::copy;
use std::path::PathBuf;

fn main() {
    // Allows to show relevant environment variables for debugging purpose
    print_env();

    let target = env::var("TARGET").unwrap_or_default();
    let target_env = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    let target_family = env::var("CARGO_CFG_TARGET_FAMILY").unwrap_or_default();
    let out_dir = env::var("OUT_DIR").unwrap();

    let mut b = freertos_cargo_build::Builder::new();

    b.freertos("FreeRTOS-Kernel/");

    if target == "thumbv7em-none-eabihf" {
        b.freertos_config("examples/stm32-cortex-m4-f401re");
        copy(
            "examples/stm32-cortex-m4-f401re/memory.x",
            PathBuf::from(out_dir.as_str()).join("memory.x"),
        )
        .unwrap();
    }

    b.compile().unwrap_or_else(|e| panic!("{}", e));
}

/// Print relevant environment variables.
/// To avoid cluttering the output on each build, this is not displayed in the terminal.
/// See the output in the corresponding target output file e.g. target/debug/build/<pkg>/output
fn print_env() {
    let env_keys = ["TARGET", "OUT_DIR", "HOST"];
    env::vars().for_each(|(key, val)| {
        if key.starts_with("CARGO") {
            println!("{}={}", key, val);
        } else if env_keys.contains(&key.as_str()) {
            println!("{}={}", key, val);
        } else {
            // println!("{}={}", key, val);
        }
    });
}
