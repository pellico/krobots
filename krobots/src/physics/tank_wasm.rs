use bevy::ecs::schedule::common_conditions;
use bevy::math::bool;
use futures::future::{BoxFuture, FutureExt};

use crate::physics::tank_wasm::krobots::krobots::tank::PolarVector;
use crate::physics::{PhysicsEngine, Real, Rotation2, Tank, Vector2};
use core::num;
use std::path::PathBuf;
use indexmap::IndexMap;
use krobots::krobots::tank;
use krobots::krobots::tank::{Host, RadarResult, SimulationConfig, TankRadar, TankStatus};
use std::{
    cell::RefCell,
    default,
    ffi::OsStr,
    future::{self, Future},
    path::Path,
    pin::Pin,
    rc::Rc,
    sync::{Arc, Mutex},
};
use wasmtime::{
    component::{bindgen, Component, HasSelf, Linker},
    Engine, Store,
};
const FUEL_INTERVAL: u64 = 10000;
bindgen!({
    world:"krobot",
    async: {
        only_imports:[]
    },
     require_store_data_send: true,
      //additional_derives: [Default],
    include_generated_code_from_file: false,

});
impl Default for tank::TankStatus {
    fn default() -> Self {
        Self {
            angle: Default::default(),
            angvel: Default::default(),
            cannon_angle: Default::default(),
            cannon_temp: Default::default(),
            command_result: tank::CommandResult::Success,
            damage: Default::default(),
            energy: Default::default(),
            power_source: PolarVector { p: 0.0, r: 0.0 },
            radar_result: RadarResult {
                angle: 0.0,
                tanks: vec![],
            },
            tick: Default::default(),
            velocity: PolarVector { p: 0.0, r: 0.0 },
        }
    }
}

impl TankStatus {
    fn update_tank_status(&mut self, p_engine: &PhysicsEngine, tank_index: usize) {
        let tank = p_engine.tank(tank_index);
        let tank_position = tank.position();
        let vel = tank.linvel();
        let angvel = tank.angular_velocity();

        let (angle, detected_tanks) = p_engine.get_radar_result(tank_index);

        let mut tanks_radar = Vec::new();
        for (tank, distance) in detected_tanks {
            let t_radar = TankRadar {
                damage: tank.damage,
                distance,
            };
            tanks_radar.push(t_radar);
        }
        tanks_radar.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        tanks_radar.truncate(10);

        let radar_result = tank::RadarResult {
            angle,
            tanks: tanks_radar,
        };

        // Command result is not updated because it will be done during command processing
        self.tick = p_engine.tick();
        self.velocity = krobots::krobots::tank::PolarVector {
            r: vel.norm(),
            p: vel.y.atan2(vel.x),
        };

        self.angle = tank_position.rotation.angle();
        self.angvel = angvel;
        self.energy = tank.energy();
        self.damage = tank.damage();
        self.cannon_angle = tank.turret().angle();
        self.power_source = krobots::krobots::tank::PolarVector {
            r: tank_position.translation.vector.norm(),
            p: Rotation2::rotation_between(
                &Vector2::<Real>::x(),
                &(-tank_position.translation.vector),
            )
            .angle(),
        };

        self.cannon_temp = tank.turret().cannon_temperature();
        self.radar_result = radar_result;
    }
}

struct MyState {
    command: Option<tank::Command>,
    simulation_config: tank::SimulationConfig,
    tank_status: tank::TankStatus,
}

impl krobots::krobots::tank::Host for Arc<Mutex<MyState>> {
    fn get_simulation_config(&mut self) -> SimulationConfig {
        let state = self.lock().unwrap();
        state.simulation_config
    }
    fn get_status(&mut self) -> TankStatus {
        let state = self.lock().unwrap();
        state.tank_status.clone()
    }
    fn execute_command(&mut self, command: tank::Command) -> () {
        let mut state = self.lock().unwrap();
        state.command = Some(command);
        state.tank_status.command_result = tank::CommandResult::Pending;
    }

    // #[doc = " Get tank status"]
    // fn get_status(&mut self) -> TankStatus {
    //     let p_engine = self.p_engine.lock().unwrap();
    //     let tank = p_engine.tank(self.tank_index);
    //     let tank_position = tank.position();
    //     let vel = tank.linvel();
    //     let angvel = tank.angular_velocity();
    //     TankStatus {
    //         tick: p_engine.tick(),
    //         velocity: krobots::krobots::tank::PolarVector {
    //             r: vel.norm(),
    //             p: vel.y.atan2(vel.x),
    //         },

    //         angle: tank_position.rotation.angle(),
    //         angvel,
    //         energy: tank.energy(),
    //         damage: tank.damage(),
    //         cannon_angle: tank.turret().angle(),
    //         power_source: krobots::krobots::tank::PolarVector {
    //             r: tank_position.translation.vector.norm(),
    //             p: Rotation2::rotation_between(
    //                 &Vector2::<Real>::x(),
    //                 &(-tank_position.translation.vector),
    //             )
    //             .angle(),
    //         },
    //         success: true,
    //         cannon_temp: tank.turret().cannon_temperature(),
    //     }
    // }

    // fn get_simulation_config(&mut self) -> SimulationConfig {
    //     let p_engine = self.p_engine.lock().unwrap();
    //     SimulationConfig {
    //         tank_energy_max: p_engine.conf.tank_energy_max,
    //         damage_max: p_engine.conf.damage_max,
    //         bullet_max_range: p_engine.conf.bullet_max_range,
    //         zero_power_limit: p_engine.conf.zero_power_limit,
    //         radar_angle_increment_max: p_engine.conf.radar_angle_increment_max,
    //         radar_width_max: p_engine.conf.radar_width_max,
    //         radar_max_detection_range: p_engine.conf.radar_max_detection_distance,
    //         bullet_speed: p_engine.conf.bullet_speed,
    //         max_forward_power: p_engine.conf.tank_engine_power_max,
    //         max_turning_power: p_engine.conf.turning_power_max,
    //     }
    // }
}

pub struct WasmTanks {
    engine: Engine,
    tanks: IndexMap<String, WasmTank>,
}

impl Default for WasmTanks {
    fn default() -> Self {
        // Instantiate the engine and store
        let mut config = wasmtime::Config::new();
        config.consume_fuel(true);
        config.async_support(true);
        config.debug_info(true);
        let engine = Engine::new(&config).expect("Failed instantiating engine");
        WasmTanks {
            tanks: Default::default(),
            engine,
        }
    }
}

impl WasmTanks {
    pub fn new_tank<P: AsRef<Path>>(
        &mut self,
        path: P,
        name: &str,
        p_engine: &mut PhysicsEngine,
        num_tanks: usize,
    ) -> anyhow::Result<()> {
        let component = Component::from_file(&self.engine, path)?;
        let mut linker = Linker::new(&self.engine);
        krobots::krobots::tank::add_to_linker::<_, HasSelf<_>>(
            &mut linker,
            |state: &mut Arc<Mutex<MyState>>| state,
        )?;
        let tank_index = p_engine.add_tank_in_circle(name.to_string(), num_tanks);

        let simulation_config = SimulationConfig {
            tank_energy_max: p_engine.conf.tank_energy_max,
            damage_max: p_engine.conf.damage_max,
            bullet_max_range: p_engine.conf.bullet_max_range,
            zero_power_limit: p_engine.conf.zero_power_limit,
            radar_angle_increment_max: p_engine.conf.radar_angle_increment_max,
            radar_width_max: p_engine.conf.radar_width_max,
            radar_max_detection_range: p_engine.conf.radar_max_detection_distance,
            bullet_speed: p_engine.conf.bullet_speed,
            max_forward_power: p_engine.conf.tank_engine_power_max,
            max_turning_power: p_engine.conf.turning_power_max,
        };
        let mut tank_status: tank::TankStatus = Default::default();
        tank_status.update_tank_status(p_engine, tank_index);

        let state = Arc::new(Mutex::new(MyState {
            command: None,
            simulation_config,
            tank_status,
        }));
        let mut store = Store::new(&self.engine, state.clone());
        // store.call_hook(|mut cx, call_hook| {
        //     match call_hook {
        //         wasmtime::CallHook::CallingHost => {
        //             println!("CallingHost");
        //         }
        //         wasmtime::CallHook::CallingWasm => {
        //             println!("CallingWasm");
        //         }
        //         wasmtime::CallHook::ReturningFromHost => {
        //             println!("ReturningFromHost");
        //         }
        //         wasmtime::CallHook::ReturningFromWasm => {
        //             println!("ReturningFromWasm");
        //         }
        //     };
        //     Ok(())
        // });
        store
            .set_fuel(1000000000000000)
            .expect("Failed to set fuel");
        store
            .fuel_async_yield_interval(Some(FUEL_INTERVAL))
            .expect("Failed to set yield interval");
        let bindings = futures::executor::block_on(Krobot::instantiate_async(
            &mut store, &component, &linker,
        ))?;
        let bindings = Box::pin(bindings);

        //let a =   self.tanks[name].bindings.call_run(store).boxed();

        let new_tank = WasmTankBuilder {
            bindings,
            state,
            index: tank_index,
            future_builder: |bind: &Pin<Box<Krobot>>| bind.call_run(store).boxed(),
        }
        .build();
        self.tanks.insert(name.to_string(), new_tank);

        Ok(())
    }

    pub fn new(p_engine: &mut PhysicsEngine) -> WasmTanks {
        let mut result = WasmTanks::default();
        let num_tanks = p_engine.conf.tanks_list.len();
        p_engine.conf.tanks_list.clone().iter().for_each(|path_wasm| {
            let tank_name = path_wasm.file_name().unwrap().to_str().unwrap();
            result
                .new_tank(path_wasm, tank_name, &mut *p_engine, num_tanks)
                .unwrap()
        });
        result
    }
    pub fn next_step(&mut self, p_engine: &mut PhysicsEngine) -> anyhow::Result<()> {
        for tank in self.tanks.values_mut() {
            tank.next_step(p_engine)?;
        }
        Ok(())
    }

    fn get_tank_mut<T: AsRef<str>>(&mut self, name: T) -> anyhow::Result<&mut WasmTank> {
        self.tanks
            .get_mut(name.as_ref())
            .ok_or_else(|| anyhow::anyhow!("No item"))
    }
}



use ouroboros::self_referencing;
#[self_referencing]
pub struct WasmTank {
    bindings: Pin<Box<Krobot>>,
    index: usize,
    state: Arc<Mutex<MyState>>,
    #[borrows(bindings)]
    #[not_covariant]
    future: BoxFuture<'this, wasmtime::Result<()>>,
}

fn process_command(
    p_engine: &mut PhysicsEngine,
    tank_index: usize,
    command: tank::Command,
) -> tank::CommandResult {
    let mut tank = p_engine.tank_mut(tank_index);
    match command {
        tank::Command::FireCannon => match tank.turret_mut().fire() {
            true => tank::CommandResult::Success,
            false => tank::CommandResult::Fail,
        },
        tank::Command::SetCannotPosition(angle) => {
            tank.turret_mut().set_cannon_position(angle);
            tank::CommandResult::Success
        }
        tank::Command::SetEnginePower((fraction_forward_power, fraction_turning_power)) => {
            tank.set_engine_power(fraction_forward_power);
            tank.set_turning_power(fraction_turning_power);
            tank::CommandResult::Success
        }
        tank::Command::SetRadar((radar_increment, radar_width)) => {
            match p_engine.set_radar_position(tank_index, radar_increment, radar_width) {
                true => tank::CommandResult::Success,
                false => tank::CommandResult::Fail,
            }
        }
    }
}

impl WasmTank {
    pub fn next_step(&mut self, p_engine: &mut PhysicsEngine) -> anyhow::Result<bool> {
        let tank_index = *self.borrow_index();
        // Update state before executing the next step
        let mut state = self.borrow_state().lock().unwrap();
        state.command = None;
        state.tank_status.update_tank_status(p_engine, tank_index);
        drop(state);
        use futures::task::Waker;
        let waker = Waker::noop();
        let mut cx = std::task::Context::from_waker(waker);
        let poll_result = self.with_future_mut(|future| future.poll_unpin(&mut cx));
        let result = match poll_result {
            std::task::Poll::Pending => Ok(false),
            std::task::Poll::Ready(val) => {
                panic!();
                val.map(|_| true)
            }
        };

        // Process command
        let mut state = self.borrow_state().lock().unwrap();
        state.tank_status.command_result = match state.command {
            None => tank::CommandResult::Success,
            Some(command) => {
                state.command = None;
                process_command(p_engine, tank_index, command)
            }
        };
        result
    }
}

#[cfg(test)]
mod tests {
    use crate::Opts;

    use super::*;
    #[test]
    fn simple_tank() {
        let mut physics_engine = PhysicsEngine::default();
        let mut all_tanks = WasmTanks::default();
        all_tanks
            .new_tank(
                "../simple_tank/target/wasm32-unknown-unknown/debug/simple_tank_comp.wasm",
                "simple-tank",
                &mut physics_engine,
                1,
            )
            .unwrap();
        let tank = all_tanks.get_tank_mut("simple-tank").unwrap();
        for _ in 0..10 {
            println!("next step");
            assert!(!tank.next_step(&mut physics_engine).unwrap());
        }
    }
}
