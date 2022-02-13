pub use super::tank::{Bullet, Tank};
pub use super::{PhysicsEngine,SimulationState};

#[derive(Debug, Clone)]
pub struct ErrorUIComm;

impl std::error::Error for ErrorUIComm {}

impl std::fmt::Display for ErrorUIComm {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Error in remote UI communication")
    }
}

#[derive(Default)]
pub struct UIGameState {
    pub tanks: Vec<Tank>,
    pub bullets: Vec<Bullet>,
    pub max_num_tanks: usize,
    pub tick: u32,
    pub max_ticks:u32,
    pub debug_mode : bool,
    pub state: SimulationState
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
    #[inline]
    fn create_state(&self,state: &PhysicsEngine) -> UIGameState {
        UIGameState {
            tanks : state.tanks.clone(),
            bullets : state.bullets.clone(),
            max_num_tanks : state.max_num_tanks,
            tick : state.tick,
            max_ticks : state.max_ticks,
            debug_mode : state.debug_mode,
            state: state.state
        }
    }
    fn send(&self, state: &PhysicsEngine) -> Result<(), ErrorUIComm>;
}

pub trait GameStateReceiver: Send {
    fn receiver(&self) -> Option<UIGameState>;
}
