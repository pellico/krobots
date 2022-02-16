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
mod local_ch;
use clap::{Parser};
use ktanks_server::ui;
use ktanks_server::physics::{PhysicsEngine};
use ktanks_server::Opts;
use ktanks_server::remote_ch;



fn main() {

    let opts: Opts = crate::Opts::parse();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(opts.log_level.clone())).init();
    if opts.no_gui     {
        let tx_state = remote_ch::UISender::new(opts.remote_gui_port);
        let rx_ui_command = remote_ch::CommandReceiver::new();
        let handle = PhysicsEngine::new(&opts,Box::new(tx_state),Box::new(rx_ui_command));
        handle.join().expect("Failed simulation end");
    }
    else
    {
        let (tx_state, rx_state) = local_ch::create_state_channels();
        let (tx_ui_command,rx_ui_command) =local_ch::create_command_channels();
        PhysicsEngine::new(&opts,Box::new(tx_state),Box::new(rx_ui_command));
        ui::start_gui(Box::new(rx_state),Box::new(tx_ui_command));
    }

    } 
