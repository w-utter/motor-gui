extern crate protobuf_codegen;
use protobuf_codegen::Codegen;

fn main() {
    Codegen::new()
        .cargo_out_dir("protos")
        .include("src")
        .input("src/protos/motor.proto")
        .run_from_script()
}
