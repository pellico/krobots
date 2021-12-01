mod krobots_main;
mod physics;
mod conf;
mod input;
mod networking;
use clap::{AppSettings, Parser};
// Include the `items` module, which is generated from items.proto.
pub mod tank_proto {
    include!(concat!(env!("OUT_DIR"), "/protobuffer.tank.rs"));
}
use macroquad::prelude::*;


/// Run krobots server
#[derive(Parser)]
#[clap(version = "0.1", author = "Oreste Bernardi <oreste@oreste.eu>")]
struct Opts {
    /// How manyt tanks in this game
    num_tanks: u8,
    /// Port used to register new tanks
    #[clap(short, long, default_value = "55230")]
    port: u16

}

fn window_conf() -> Conf {
    Conf {
        window_title: "ETank".to_owned(),
        window_width: 800,
        window_height: 600,
        //fullscreen: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let opts: Opts = Opts::parse();
    env_logger::init();
    krobots_main::main(opts.num_tanks,opts.port).await;
   // graphics::main().await;   

}