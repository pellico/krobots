use crate::physics::*;
use message_io::network::{NetEvent, Transport};
use message_io::node::{self, NodeHandler, NodeListener};
use std::sync::mpsc;
use log::{debug, error,warn, info};
pub struct UISender {
    handler: NodeHandler<()>,
    listener: NodeListener<()>,
    connected:bool
}

impl GameStateSender for UISender {
    #[inline]
    fn send(&self, state: &PhysicsEngine) -> Result<(), ErrorUIComm> {
        let state_to_transfer = self.create_state(state);
        // Read incoming network events.
        self.listener.for_each(|event| match event.network() {
            NetEvent::Connected(_, _) => self.connected=true,
            NetEvent::Accepted(_endpoint, _listener) => println!("Client connected"), // Tcp or Ws
            NetEvent::Message(endpoint, data) => {
                println!("Received: {}", String::from_utf8_lossy(data));
                self.handler.network().send(endpoint, data);
            }
            NetEvent::Disconnected(_endpoint) => println!("Client disconnected"), //Tcp or Ws
        });
    }
}

const transport :Transport = Transport::FramedTcp;

impl UISender {
    pub fn new(port: u16) -> Self {
        let addr = "0.0.0.0:3042";
        let (handler, listener) = node::split::<()>();
        match handler.network().listen(Transport::FramedTcp, addr) {
            Ok((_id, real_addr)) => info!("Server running at {} by {}", real_addr, transport),
            Err(_) => return error!("Can not listening at {} by {}", addr, transport),

        }
             
        UISender {
            handler: handler,
            listener: listener,
            connected: false;
        }
    }
}

pub struct UIReceiver {
    rx_data: mpsc::Receiver<UIGameState>,
}

impl GameStateReceiver for UIReceiver {
    #[inline]
    fn receiver(&self) -> Option<UIGameState> {
        // Keep just last data in queue
        self.rx_data.try_iter().last()
    }
}

pub struct CommandSender;
impl UICommandSender for CommandSender {
    #[inline]
    fn send(&self, _: UICommand) -> Result<(), ErrorUIComm> {
        Ok(())
    }
}

pub struct CommandReceiver;
impl UICommandReceiver for CommandReceiver {
    #[inline]
    fn receive(&self) -> Option<UICommand> {
        None
    }
}
