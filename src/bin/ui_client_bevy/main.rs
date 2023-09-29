use clap::{Parser};
use ktanks_server::remote_ch::*;
use ktanks_server::enable_human_panic;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Run krobots ui client
#[derive(Parser)]
#[clap(version = VERSION, author = "Oreste Bernardi <oreste@oreste.eu>")]
pub struct Opts {
    /// ipv4 address
    #[clap(default_value = "127.0.0.1")]
    ip: String,
    // Server port
    #[clap(default_value = "3042")]
    port: u16,
    //Log level to be used if environmental variable RUST_LOG is not set.
    #[clap(short, long, default_value = "info",value_parser=["error","warn","info","debug","trace"])]
    log_level: String,
    /// Scaling factor of ui graphics compared to physical simulation dimension.
    /// Increment teh value to see bigger graphics compared to physical collider
    #[clap(long, default_value = "0.15")]
    pub graphics_scaling_factor: f32,
}


fn main() {
    enable_human_panic();
    let opts: Opts = crate::Opts::parse();
    let rx_state = UIReceiver::new(&opts.ip,opts.port);
    let tx_ui_command=CommandSender;
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level.clone())).init();
    #[cfg(feature = "bevy")]
    ktanks_server::ui_bevy::start_gui(Box::new(rx_state),Box::new(tx_ui_command),1.0/opts.graphics_scaling_factor);
    #[cfg(feature = "macroquad")]
    ktanks_server::ui::start_gui(Box::new(rx_state),Box::new(tx_ui_command),1.0/opts.graphics_scaling_factor);
}