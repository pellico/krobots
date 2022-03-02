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
use std::collections::HashSet;
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
    pub fn new(port: u16,max_number_connections:usize) -> Self {
        let addr = "0.0.0.0";
        let (handler, listener) = node::split::<UIGameState>();
        let handler_copy = handler.clone();
        match handler.network().listen(Transport::FramedTcp, (addr,port)) {
            Ok((_id, real_addr)) => info!("Waiting ui client connection at {}", real_addr),
            Err(_) => {
                error!("Can not listening at {} by {}", addr, TRANSPORT);
                panic!("Not able to listen connection");
            }
        }
        thread::spawn(move || {
            let mut endpoints = HashSet::with_capacity(max_number_connections);
            listener.for_each(move |event| match event {
                NodeEvent::Network(net_event) => match net_event {
                    NetEvent::Connected(_, _) => (),
                    NetEvent::Accepted(endpoint, _listener) => {
                       // Refuse connections if max_number_connections is exceeded
                       if endpoints.len() >= max_number_connections {
                           handler.network().remove(endpoint.resource_id());
                        } else {
                            endpoints.insert(endpoint);
                        }
                    },
                    NetEvent::Message(_, _) => (),
                    NetEvent::Disconnected(endpoint) => {
                        info!("Client @ {} disconnected",endpoint.addr());
                        endpoints.remove(&endpoint);
                        
                },
                },
                NodeEvent::Signal(signal) => {
                    let data = bincode::serialize(&signal)
                            .expect("Unable to serialize game state for UI");
                    for endpoint in endpoints.iter() {
                        match handler.network().is_ready(endpoint.resource_id()) {
                            Some(true) => {handler.network().send(*endpoint,&data);},
                            Some(false) => continue,
                            None => continue,
                        }
                    }
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
    receiver: EventReceiver<StoredNodeEvent<()>>,
    remote_addr : String,
    _task: NodeTask, // Keep it to avoid drop of it.
}

impl UIReceiver {
    pub fn new(remote_ip:&str, remote_port:u16) -> Self {
        let (handler, listener) = node::split::<()>();
        let (task, receiver) = listener.enqueue();
        let (server_id, local_addr) = handler
            .network()
            .connect(TRANSPORT, (remote_ip,remote_port))
            .unwrap();
        UIReceiver {
            handler: handler,
            server_id: server_id,
            local_addr: local_addr,
            receiver: receiver,
            remote_addr : remote_ip.to_string(),
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
                                "Can not connect to server at {}",self.remote_addr
                            )
                        }
                    }
                    StoredNetEvent::Accepted(_, _) => unreachable!(), // Only generated when a listener accepts
                    StoredNetEvent::Message(_endpoint, input_data) => {
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
    /// Remote UI cannon send any command to the server.
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
    /// No command are received from UI client
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
