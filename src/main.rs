mod krobots_main;
mod physics;
mod conf;
mod input;
mod networking;
use clap::{Parser};
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
    port: u16,
    //Log level to be used if enviromental variable RUST_LOG is not set.
    #[clap(short, long, default_value = "info",possible_values=["error","warn","info","debug","trace"])]
    log_level: String,
    /// Max number of simulation step. If 0 no end until only one survived.
    #[clap(short, long, default_value = "0")]
    max_steps: u32,

}

fn window_conf() -> Conf {
    Conf {
        window_title: "ETank".to_owned(),
        window_width: 1024,
        window_height: 768,
        //fullscreen: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let opts: Opts = Opts::parse();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level)).init();
    krobots_main::main(opts.num_tanks,opts.port,opts.max_steps).await;
   // graphics::main().await;   

}