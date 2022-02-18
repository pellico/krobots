use super::*;
use std::net::{UdpSocket,Ipv4Addr};
use log::{debug};

pub (super) struct ClientInterface{
    socket : UdpSocket,
    connection_port_counter : u16,
    debug_mode : bool
}

pub (super) struct TankClientConnection {
    socket : UdpSocket,

}

impl ClientConnection for TankClientConnection {
    fn recv(&mut self,buffer :&mut [u8]) -> io::Result<usize> {
        self.socket.recv(buffer) 

    }
    fn send(&mut self,buffer :&[u8]) -> io::Result<usize> {
        self.socket.send(buffer)
    }
}

pub (super) fn new(port:u16,debug_mode:bool)->ClientInterface {
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port)).expect("Not able to open socket: {}");
    ClientInterface {
        socket : socket,
        connection_port_counter : port,
        debug_mode : debug_mode
    }
}

impl NetInterface for ClientInterface {

    fn wait_new_tank(&mut self,buffer :&mut [u8]) -> (Box<dyn ClientConnection>,usize) {
        let (amt, src) = self.socket.recv_from(buffer).expect("Not received data");
        debug!("Connection request form from {}", src);
        //Create connection and store connection data
        self.connection_port_counter += 1;
        let dedicated_socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, self.connection_port_counter))
            .expect("Not able to open socket");
        dedicated_socket.connect(src).unwrap();           
        
        if self.debug_mode {
            // When in debug mode we want to wait for tank client Command
            // but we would like to be able to kill process as well.
            dedicated_socket.set_nonblocking(false).unwrap();
            dedicated_socket.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        } else {
            dedicated_socket.set_nonblocking(true).unwrap();
        }

        (Box::new(TankClientConnection {
            socket : dedicated_socket,
        }),amt)

    }

}




