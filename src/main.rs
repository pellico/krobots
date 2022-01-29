/*
krobots
Copyright (C) 2021  Oreste Bernardi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
mod ui;
mod physics;
mod conf;
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


fn main() {
    let opts: Opts = Opts::parse();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level)).init();
    ui::start_gui(opts.num_tanks,opts.port,opts.max_steps);

}