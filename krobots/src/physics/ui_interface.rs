pub use super::tank::{Bullet, Tank};
pub use super::{PhysicsEngine, SimulationState};
use serde::{Deserialize, Serialize};
#[derive(Debug, Clone)]
pub struct ErrorUIComm;

impl std::error::Error for ErrorUIComm {}

impl std::fmt::Display for ErrorUIComm {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Error in remote UI communication")
    }
}

#[derive(Default, Serialize, Deserialize)]
pub struct UIGameState {
    pub tanks: Vec<Tank>,
    pub bullets: Vec<Bullet>,
    pub tick: u32,
    pub max_ticks: u32,
    pub debug_mode: bool,
    pub state: SimulationState,
    pub zero_power_limit: f32,
}

pub enum UICommand {
    QUIT,
    EnterDebugMode,
    ExitDebugMode,
    NextStep,
}

pub trait UICommandSender: Send {
    fn send(&self, command: UICommand) -> Result<(), ErrorUIComm>;
}

pub trait UICommandReceiver: Send {
    fn receive(&self) -> Option<UICommand>;
}

pub trait GameStateSender: Send {
    #[inline]
    fn create_state(&self, state: &PhysicsEngine) -> UIGameState {
        UIGameState {
            tanks: state.tanks.clone(),
            bullets: state.bullets.clone(),
            tick: state.tick,
            max_ticks: state.max_ticks,
            debug_mode: state.debug_mode,
            state: state.state,
            zero_power_limit: state.conf.zero_power_limit,
        }
    }
    fn send(&mut self, state: &PhysicsEngine) -> Result<(), ErrorUIComm>;
}

pub trait GameStateReceiver: Send {
    fn receiver(&mut self) -> Option<UIGameState>;
}
