use super::*;
use std::io::prelude::*;
use std::net::{TcpListener,TcpStream,Ipv4Addr};
use log::{debug};

pub (super) struct ClientInterface{
    listener : TcpListener,
    debug_mode : bool
}

pub (super) struct TankClientConnection {
    stream : TcpStream,

}

impl ClientConnection for TankClientConnection {
    fn recv(&mut self,buffer :&mut [u8]) -> io::Result<usize> {
        self.stream.read(buffer) 

    }
    fn send(&mut self,buffer :&[u8]) -> io::Result<usize> {
        self.stream.write(buffer)
    }
}

impl NetInterface<TankClientConnection> for ClientInterface {
    fn new(port:u16,debug_mode:bool)->Self {
        let listener = TcpListener::bind((Ipv4Addr::UNSPECIFIED, port)).expect("Not able to open socket: {}");
        ClientInterface {
            listener : listener,
            debug_mode : debug_mode
        }
    }
    fn wait_new_tank(&mut self,buffer :&mut [u8]) -> (TankClientConnection,usize) {
        let (mut stream,src)=self.listener.accept().expect("Error in while waiting for connection for simulator");
        debug!("Connection request form from {}", src);
        //stream.set_nonblocking(false).unwrap();
        stream.set_nodelay(true).unwrap();
        let msg_size = stream.read(buffer).expect("Error while receiving tank client registration confirmation");

        if self.debug_mode {
            // When in debug mode we want to wait for tank client Command
            // but we would like to be able to kill process as well.
            stream.set_nonblocking(false).unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        } else {
            stream.set_nonblocking(true).unwrap();
        }

        (TankClientConnection {
            stream : stream,
        },msg_size)

    }

}