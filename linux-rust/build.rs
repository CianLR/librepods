// Generates Rust types for the AACP sensor data (RTBuddy) protobuf messages.
// protox parses the .proto in pure Rust, so building doesn't need protoc installed.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=proto/rtbuddy.proto");

    let file_descriptors = protox::compile(["rtbuddy.proto"], ["proto"])?;
    prost_build::Config::new().compile_fds(file_descriptors)?;
    Ok(())
}
