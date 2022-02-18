use clap::{Parser};
use ktanks_server::remote_ch::*;
use ktanks_server::ui;
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
    #[clap(short, long, default_value = "info",possible_values=["error","warn","info","debug","trace"])]
    log_level: String,
}


fn main() {

    let opts: Opts = crate::Opts::parse();
    let rx_state = UIReceiver::new(&opts.ip,opts.port);
    let tx_ui_command=CommandSender;
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level.clone())).init();
    ui::start_gui(Box::new(rx_state),Box::new(tx_ui_command));

}