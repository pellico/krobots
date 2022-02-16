pub mod remote_ch;
pub mod ui;
pub mod physics;
pub mod conf;
// Include the `items` module, which is generated from items.proto.
pub mod tank_proto {
    include!(concat!(env!("OUT_DIR"), "/protobuffer.tank.rs"));
}

use clap::{Parser};
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Run krobots server
#[derive(Parser)]
#[clap(version = VERSION, author = "Oreste Bernardi <oreste@oreste.eu>")]
pub struct Opts {
    /// How many tanks in this game
    pub (crate) num_tanks: usize,
    /// Port used to register new tanks
    #[clap(short, long, default_value = "55230")]
    pub (crate) port: u16,
    //Log level to be used if environmental variable RUST_LOG is not set.
    #[clap(short, long, default_value = "info",possible_values=["error","warn","info","debug","trace"])]
    pub log_level: String,
    /// Max number of simulation step. If 0 no end until only one survived.
    #[clap(short, long, default_value = "0")]
    pub (crate) max_steps: u32,
    /// Simulator stop waiting for command from tank client
    #[clap(short, long)]
    pub (crate) debug_mode: bool,
    /// Simulation step x sec. This has no relation with ui frame rate.
    #[clap(long, default_value = "60.0")]
    pub (crate) sim_step_rate: f64,
    /// Remote gui port.
    #[clap(long, default_value = "3042")]
    pub remote_gui_port: u16,
    /// Headless server When specified server will not show any ui but it expect a connection from ui_client
    #[clap(long)]
    pub no_gui: bool,

}