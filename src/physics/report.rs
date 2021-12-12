use super::{Tank};
use std::fs::{File};
use csv;
fn save_tank_report(path:String,tanks:&Vec<Tank>) -> std::io::Result<()> {
    let mut file = File::open(path)?;
    for tank in tanks {

    }
    Ok(())
}