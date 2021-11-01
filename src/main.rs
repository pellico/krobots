mod graphics;
mod krobots_main;
mod physics;
mod conf;
mod input;
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
    krobots_main::main().await;
   // graphics::main().await;   

}