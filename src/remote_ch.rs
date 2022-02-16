use crate::physics::*;
use message_io::events::EventReceiver;
use message_io::network::{Endpoint, NetEvent, Transport};
use message_io::node::{
    self, NodeHandler, NodeEvent, NodeTask, StoredNetEvent, StoredNodeEvent,
};
use std::net::SocketAddr;
use std::{thread,time};
use std::sync::mpsc;
use std;
use bincode;
use log::{error, info,debug};

pub struct UISender {
    handler : NodeHandler<UIGameState>
}

impl GameStateSender for UISender {
    #[inline]
    fn send(&mut self, state: &PhysicsEngine) -> Result<(), ErrorUIComm> {
        let state_to_transfer = self.create_state(state);
        self.handler.signals().send(state_to_transfer);
        Ok(())
    }
}

const TRANSPORT: Transport = Transport::FramedTcp;

impl UISender {
    pub fn new(port: u16) -> Self {
        let addr = "0.0.0.0";
        let (handler, listener) = node::split::<UIGameState>();
        let handler_copy = handler.clone();
        match handler.network().listen(Transport::FramedTcp, (addr,port)) {
            Ok((_id, real_addr)) => info!("Waiting ui client connection at {} by {}", real_addr, TRANSPORT),
            Err(_) => {
                error!("Can not listening at {} by {}", addr, TRANSPORT);
                panic!("Not able to listen connection");
            }
        }
        thread::spawn(move || {
            let mut endpoint_stored:Option<Endpoint> = None;
            let mut state = UIGameState::default();
            listener.for_each(move |event| match event {
                NodeEvent::Network(net_event) => match net_event {
                    NetEvent::Connected(_, _) => (),
                    NetEvent::Accepted(endpoint, _listener) => {
                        match endpoint_stored {
                            Some(_) => {handler.network().remove(endpoint.resource_id());},
                            None => endpoint_stored = Some(endpoint),
                        };
                    }, // Tcp or Ws
                    NetEvent::Message(endpoint, _) => {
                        
                        let data = bincode::serialize(&state)
                            .expect("Unable to serialize game state for UI");
                        handler.network().send(endpoint, &data);
                    },
                    NetEvent::Disconnected(_endpoint) => {
                        endpoint_stored =  None;
                        info!("Client disconnected")
                },
                },
                NodeEvent::Signal(signal) => {
                    state = signal;
                }
            })
        });

        UISender {
            handler : handler_copy

        }
    }
}

pub struct UIReceiver {
    handler: NodeHandler<()>,
    server_id: Endpoint,
    local_addr: SocketAddr,
    remote_addr: String,
    receiver: EventReceiver<StoredNodeEvent<()>>,
    _task: NodeTask, // Keep it to avoid drop of it.
}

impl UIReceiver {
    pub fn new(remote_addr: &str) -> Self {
        let (handler, listener) = node::split::<()>();
        let (task, receiver) = listener.enqueue();
        let (server_id, local_addr) = handler
            .network()
            .connect(TRANSPORT, remote_addr.clone())
            .unwrap();
        UIReceiver {
            handler: handler,
            server_id: server_id,
            local_addr: local_addr,
            remote_addr: remote_addr.to_string(),
            receiver: receiver,
            _task: task,
        }
    }
}

impl GameStateReceiver for UIReceiver {
    #[inline]
    fn receiver(&mut self) -> Option<UIGameState> {
        let mut result = None;
        loop {
            match self.receiver.try_receive() {
                Some(event) => match event.network() {
                    StoredNetEvent::Connected(endpoint, established) => {
                        if established {
                            info!(
                                "Connected to server at {} by {}",
                                self.server_id.addr(),
                                TRANSPORT
                            );
                            info!(
                                "Client identified by local port: {}",
                                self.local_addr.port()
                            );
                            self.handler.network().send(endpoint,"start".as_bytes());
                        } else {
                            println!(
                                "Can not connect to server at {} by {}",
                                &self.remote_addr, TRANSPORT
                            )
                        }
                    }
                    StoredNetEvent::Accepted(_, _) => unreachable!(), // Only generated when a listener accepts
                    StoredNetEvent::Message(endpoint, input_data) => {
                        self.handler.network().send(endpoint,"start".as_bytes());
                        let message: UIGameState = bincode::deserialize(&input_data).unwrap();
                        result = Some(message);
                    }
                    StoredNetEvent::Disconnected(_) => {
                        // On server disconnection kill application
                        info!("Server is disconnected probably ended the simulation");
                        self.handler.stop();
                        std::process::exit(0);
                    }
                },
                None => break,
            }
        }
        result
    }
}

pub struct CommandSender;
impl UICommandSender for CommandSender {
    #[inline]
    fn send(&self, _: UICommand) -> Result<(), ErrorUIComm> {
        Ok(())
    }
}

pub struct CommandReceiver{
    rx_command : mpsc::Receiver<UICommand>
}
impl UICommandReceiver for CommandReceiver {
    #[inline]
    fn receive(&self) -> Option<UICommand> {
        match self.rx_command.try_recv() {
            Ok(a) => Some(a),
            Err(_) => None
        }
    }
}

impl CommandReceiver {
    /// Install Ctrl-C control handle for smooth exiting
    pub fn new() -> Self{

        let (tx_data, rx_data) = mpsc::channel::<UICommand>();
        ctrlc::set_handler(move || {
            debug!("Received Ctrl-C quit application");
            tx_data.send(UICommand::QUIT).expect("Failed to send quit command");
            //If not exiting in regular way just quit application
            thread::sleep(time::Duration::from_secs(4));
            error!("Failed to exit in regular way by generating game report. Forcing application exit");
            std::process::exit(-1);
        }).expect("Error setting Ctrl-C handler");

        CommandReceiver{
            rx_command: rx_data,
        }
    }
}
