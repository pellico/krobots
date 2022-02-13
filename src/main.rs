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
mod local_ch;
use clap::{Parser};
// Include the `items` module, which is generated from items.proto.
pub mod tank_proto {
    include!(concat!(env!("OUT_DIR"), "/protobuffer.tank.rs"));
}
use macroquad::prelude::*;
use physics::{PhysicsEngine};

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Run krobots server
#[derive(Parser)]
#[clap(version = VERSION, author = "Oreste Bernardi <oreste@oreste.eu>")]
pub struct Opts {
    /// How many tanks in this game
    num_tanks: usize,
    /// Port used to register new tanks
    #[clap(short, long, default_value = "55230")]
    port: u16,
    //Log level to be used if environmental variable RUST_LOG is not set.
    #[clap(short, long, default_value = "info",possible_values=["error","warn","info","debug","trace"])]
    log_level: String,
    /// Max number of simulation step. If 0 no end until only one survived.
    #[clap(short, long, default_value = "0")]
    max_steps: u32,
    /// Simulator stop waiting for command from tank client
    #[clap(short, long)]
    debug_mode: bool,
    /// Simulation step x sec. This has no relation with ui frame rate.
    #[clap(long, default_value = "60.0")]
    sim_step_rate: f64,

}


fn main() {

    let opts: Opts = Opts::parse();
    let (tx_state, rx_state) = local_ch::create_state_channels();
    let (tx_ui_command,rx_ui_command) =  local_ch::create_command_channels();
    PhysicsEngine::new(&opts,Box::new(tx_state),Box::new(rx_ui_command));

    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level.clone())).init();
    ui::start_gui(Box::new(rx_state),Box::new(tx_ui_command));

}