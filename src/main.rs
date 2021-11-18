mod krobots_main;
mod physics;
mod conf;
mod input;
mod networking;
// Include the `items` module, which is generated from items.proto.
pub mod tank_proto {
    include!(concat!(env!("OUT_DIR"), "/protobuffer.tank.rs"));
}
use macroquad::prelude::*;

fn window_conf() -> Conf {
    Conf {
        window_title: "ETank".to_owned(),
        fullscreen: true,
        ..Default::default()
    }
}

#[macroquad::main("window_conf")]
async fn main() {
    env_logger::init();
    krobots_main::main().await;
   // graphics::main().await;   

}