pub mod conf;
pub mod physics;
pub mod remote_ch;
#[cfg(feature = "macroquad")]
pub mod ui;
#[cfg(feature = "bevy")]
pub mod ui_bevy;
// Include the `items` module, which is generated from items.proto.
pub mod tank_proto {
    include!(concat!(env!("OUT_DIR"), "/protobuffer.tank.rs"));
}
use clap::Parser;
use std::sync::atomic::{AtomicBool, Ordering};

// Flag to true to signal to all thread to exit
static EXIT_SIGNAL: AtomicBool = AtomicBool::new(false);

#[inline]
/// To signal all thread to exit
/// used to exit from application
pub fn signal_exit() {
    EXIT_SIGNAL.store(true, Ordering::Release);
}

#[inline]
/// Check if it is time to exit from application
pub fn is_exit_application() -> bool {
    EXIT_SIGNAL.load(Ordering::Acquire)
}

const VERSION: &str = env!("CARGO_PKG_VERSION");
/// Run krobots server
#[derive(Parser)]
#[clap(version = VERSION, author = "Oreste Bernardi")]
pub struct Opts {
    /// How many tanks in this game
    pub(crate) num_tanks: usize,
    /// Port used to register new tanks
    #[clap(short, long, default_value = "55230")]
    pub(crate) port: u16,
    //Log level to be used if environmental variable RUST_LOG is not set.
    #[clap(short, long, default_value = "warn",value_parser=["error","warn","info","debug","trace"])]
    pub log_level: String,
    /// Max number of simulation step. If 0 no end until only one survived.
    #[clap(short, long, default_value = "0")]
    pub(crate) max_steps: u32,
    /// Simulator stop waiting for command from tank client
    #[clap(short, long)]
    pub(crate) debug_mode: bool,
    /// Simulation step x sec. This has no relation with ui frame rate.
    #[clap(long, default_value = "60.0")]
    pub(crate) sim_step_rate: f64,
    /// Remote gui client port. Effective only if --no_gui is used.
    #[clap(long, default_value = "3042")]
    pub remote_gui_port: u16,
    /// Maximum number supported of remote GUI Effective only if --no_gui is used.
    #[clap(long, default_value = "1")]
    pub max_num_remote_gui: u8,
    /// Headless server When specified server will not show any ui but it expect a connection from ui_client
    #[clap(long)]
    pub no_gui: bool,
    /// Tank client communication protocol. udp is faster but it has issue with NAT and firewall.
    #[clap(long, default_value = "tcp",value_parser=["tcp","udp"])]
    pub(crate) tank_client_protocol: String,
    /// Tank client communication protocol. udp is faster but it has issue with NAT and firewall.
    #[clap(long)]
    pub configuration_file: Option<String>,
    /// Scaling factor of ui graphics compared to physical simulation dimension.
    /// Increment the value to see bigger graphics compared to physical collider
    #[clap(long, default_value = "1.0")]
    pub graphics_scaling_factor: f32,
}

pub fn enable_human_panic() {
    #[allow(unused_imports)]
    use human_panic::{handle_dump, print_msg, Metadata};
    #[allow(unused_imports)]
    use std::panic::{self, PanicInfo};

    #[cfg(not(debug_assertions))]
    match ::std::env::var("RUST_BACKTRACE") {
        Err(_) => {
            let meta = Metadata {
                version: env!("CARGO_PKG_VERSION").into(),
                name: env!("CARGO_PKG_NAME").into(),
                authors: env!("CARGO_PKG_AUTHORS").replace(":", ", ").into(),
                homepage: env!("CARGO_PKG_HOMEPAGE").into(),
            };

            panic::set_hook(Box::new(move |info: &PanicInfo| {
                let file_path = handle_dump(&meta, info);
                print_msg(file_path, &meta)
                    .expect("human-panic: printing error message to console failed");
                std::process::exit(-1);
            }));
        }
        Ok(_) => {}
    }
}
