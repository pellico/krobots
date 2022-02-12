pub use super::tank::{Bullet, Tank};

#[derive(Debug, Clone)]
pub struct ErrorUIComm;

impl std::error::Error for ErrorUIComm {}

impl std::fmt::Display for ErrorUIComm {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Error in remote UI communication")
    }
}

pub enum UICommand {
    QUIT,
}

pub trait UICommandSender: Send {
    fn send(&self, command: UICommand) -> Result<(), ErrorUIComm>;
}

pub trait UICommandReceiver: Send {
    fn receive(&self) -> Option<UICommand>;
}

pub trait GameStateSender: Send {
    fn send(&self, state: (&Vec<Tank>, &Vec<Bullet>)) -> Result<(), ErrorUIComm>;
}

pub trait GameStateReceiver: Send {
    fn receiver(&self) -> Option<(Vec<Tank>, Vec<Bullet>)>;
}
