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
use std::{thread, time};

use bevy::app::TerminalCtrlCHandlerPlugin;
use clap::Parser;
use ktanks_server::physics::{PhysicsEngine, UICommand, UICommandSender};
use ktanks_server::remote_ch;
use ktanks_server::{Opts, enable_human_panic};
use ktanks_server::{conf, get_tanks_file_from_folder};
use log::{debug, error};

fn main() {
    enable_human_panic();
    let opts: Opts = crate::Opts::parse();
    env_logger::Builder::from_env(
        env_logger::Env::default()
            .default_filter_or("ktanks_server=".to_string() + &opts.log_level),
    )
    .init();
    let mut conf = match opts.configuration_file.as_ref() {
        None => conf::Conf {
            ..Default::default()
        },
        Some(path) => match conf::Conf::load_configuration(path) {
            Ok(res) => res,
            Err(err) => {
                error!("Error {} loading configuration file {}", err, path);
                std::process::exit(-1);
            }
        },
    };

    // add to conf wasm files in tank folder and sort them.
    if let Some(ref tank_folder) = opts.tank_folder {
        let tanks_in_folder = get_tanks_file_from_folder(tank_folder);
        conf.tanks_list.extend(tanks_in_folder);
        conf.tanks_list.sort();
    }
    if opts.no_gui {
        let tx_state =
            remote_ch::UISender::new(opts.remote_gui_port, opts.max_num_remote_gui as usize);
        let rx_ui_command = remote_ch::CommandReceiver::new();
        let handle = PhysicsEngine::new_simulation_thread(
            conf,
            &opts,
            Box::new(tx_state),
            Box::new(rx_ui_command),
        );
        handle.join().expect("Failed simulation end");
    } else {
        let (tx_state, rx_state) = local_ch::create_state_channels();
        let (tx_ui_command, rx_ui_command) = local_ch::create_command_channels();
        // In case of ctrl-c send a command to server thread to stop simulation and then
        // close application
        let tx_data_ctrlc = tx_ui_command.clone();
        ctrlc::set_handler(move || {
            debug!("Received Ctrl-C quit application");
            tx_data_ctrlc
                .send(UICommand::QUIT)
                .expect("Failed to send quit command");
            //If not exiting in regular way just quit application
            TerminalCtrlCHandlerPlugin::gracefully_exit();
            thread::sleep(time::Duration::from_secs(1));
            error!(
                "Failed to exit in regular way by generating game report. Forcing application exit"
            );
            std::process::exit(-1);
        })
        .expect("Error setting Ctrl-C handler");
        let handle = PhysicsEngine::new_simulation_thread(
            conf,
            &opts,
            Box::new(tx_state),
            Box::new(rx_ui_command),
        );
        ktanks_server::ui_bevy::start_gui(
            Box::new(rx_state),
            Box::new(tx_ui_command),
            opts.graphics_scaling_factor,
        );
        handle.join().expect("Failed simulation end");
    }
}
