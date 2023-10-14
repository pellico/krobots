use super::*;
use log::debug;
use std::net::{Ipv4Addr, UdpSocket};

pub(super) struct ClientInterface {
    socket: UdpSocket,
    connection_port_counter: u16,
    debug_mode: bool,
}

pub(super) struct TankClientConnection {
    socket: UdpSocket,
}

impl ClientConnection for TankClientConnection {
    fn recv(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        self.socket.recv(buffer)
    }
    fn send(&mut self, buffer: &[u8]) -> io::Result<usize> {
        self.socket.send(buffer)
    }
}

pub(super) fn new(port: u16, debug_mode: bool) -> ClientInterface {
    let socket = match UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)) {
        Ok(listener) => listener,
        Err(e) => match e.kind() {
            std::io::ErrorKind::AddrInUse => {
                error!("Port {} in use try another one", port);
                std::process::exit(-1);
            }
            _ => panic!("{}", e),
        },
    };
    ClientInterface {
        socket,
        connection_port_counter: port,
        debug_mode,
    }
}

impl NetInterface for ClientInterface {
    fn wait_new_tank(&mut self, buffer: &mut [u8]) -> (Box<dyn ClientConnection>, usize) {
        let (amt, src) = self.socket.recv_from(buffer).expect("Not received data");
        debug!("Connection request from {}", src);
        //Create connection and store connection data
        let dedicated_socket = loop {
            self.connection_port_counter += 1;
            /* If start port it is too high we risk to exceed the max number of port
            we start again from 1000. We assume that we have always enough free ports for
            all required connections.
            */
            if self.connection_port_counter == u16::MAX {
                self.connection_port_counter = 1000;
            }
            match UdpSocket::bind((Ipv4Addr::UNSPECIFIED, self.connection_port_counter)) {
                Ok(listener) => break listener,
                Err(e) => match e.kind() {
                    std::io::ErrorKind::AddrInUse => {
                        debug!(
                            "Port {} is in use try next one",
                            self.connection_port_counter
                        );
                        continue;
                    }
                    _ => panic!("{}", e),
                },
            };
        };
        info!(
            "Created connection using local port {} to {}",
            self.connection_port_counter, src
        );
        dedicated_socket.connect(src).unwrap();
        if self.debug_mode {
            // When in debug mode we want to wait for tank client Command
            // but we would like to be able to kill process as well.
            dedicated_socket.set_nonblocking(false).unwrap();
            dedicated_socket
                .set_read_timeout(Some(Duration::from_secs(3)))
                .unwrap();
        } else {
            dedicated_socket.set_nonblocking(true).unwrap();
        }

        (
            Box::new(TankClientConnection {
                socket: dedicated_socket,
            }),
            amt,
        )
    }
}
