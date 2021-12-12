use super::{Tank};
use std::fs::{File};
use csv;
pub(super) fn save_tank_report(path:&str,tanks:&Vec<Tank>) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut wtr = csv::Writer::from_writer(file);
    wtr.write_record(&["Name", "Damage", "Energy"])?;
    for tank in tanks {
        let damage = &format!("{}",tank.damage);
        let energy = &format!("{}",tank.energy);
        let name :&str = &tank.name;
        wtr.write_record(&[name, damage,energy])?;
        
    }
    wtr.flush()?;
    Ok(())
}