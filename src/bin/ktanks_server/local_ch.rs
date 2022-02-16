use std::sync::{mpsc};
use ktanks_server::physics::*;
pub struct UILocalSender {
    tx_data : mpsc::Sender<UIGameState>

}

impl GameStateSender for UILocalSender {
    #[inline]
    fn send(&mut self,state : &PhysicsEngine) -> Result<(),ErrorUIComm> {
        let state_to_transfer = self.create_state(state);
        match self.tx_data
        .send(state_to_transfer) {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorUIComm)

        }
    }
}

pub struct UILocalReceiver {
    rx_data : mpsc::Receiver<UIGameState>,

}

impl GameStateReceiver for UILocalReceiver {
    #[inline]
    fn receiver(&mut self) -> Option<UIGameState>{
        // Keep just last data in queue
        self.rx_data.try_iter().last()
    }
}

pub struct CommandLocalSender {
    tx_data : mpsc::Sender<UICommand>

}

impl UICommandSender for CommandLocalSender {
    #[inline]
    fn send(&self,command : UICommand) -> Result<(),ErrorUIComm> {
        match self.tx_data.send(command) {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorUIComm)
        }
    }
}

pub struct CommandLocalReceiver {
    rx_data : mpsc::Receiver<UICommand>,

}
impl UICommandReceiver for CommandLocalReceiver {
    #[inline]
    fn receive(&self) -> Option<UICommand> {
        match self.rx_data.try_recv() {
            Ok(a) => Some(a),
            Err(_) => None
        }
    }
}


pub fn create_state_channels() -> (UILocalSender,UILocalReceiver) {
    let (tx_data, rx_data) = mpsc::channel::<UIGameState>();
    let sender = UILocalSender {tx_data : tx_data};
    let receiver = UILocalReceiver {rx_data : rx_data};
    (sender,receiver)
}


pub fn create_command_channels() -> (CommandLocalSender,CommandLocalReceiver) {
    let (tx_data, rx_data) = mpsc::channel::<UICommand>();
    let sender = CommandLocalSender {tx_data : tx_data};
    let receiver = CommandLocalReceiver {rx_data : rx_data};
    (sender,receiver)
}
