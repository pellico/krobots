use std;

pub use super::tank::{Tank,Bullet};

#[derive(Debug,Clone)]
pub struct ErrorUIComm {}

impl std::error::Error for ErrorUIComm {
}

impl std::fmt::Display for ErrorUIComm {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Error in remote UI communication")
    }
}

pub enum UICommand {
    QUIT,
}

pub trait UICommandSender{
    fn send(&self,command : UICommand) -> Result<(),ErrorUIComm>;
}

pub trait UICommandReceiver{
    fn receive(&self) -> Result<UICommand,ErrorUIComm>;
}

pub trait GameStateSender {
    fn send(&self,state : (Vec<Tank>, Vec<Bullet>)) -> Result<(),ErrorUIComm>;
}

pub trait GameStateReceiver {
    fn receiver(&self) -> Result<(Vec<Tank>, Vec<Bullet>),ErrorUIComm>;
}