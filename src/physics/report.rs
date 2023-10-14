/*
krobots
Copyright (C) 2021  Oreste Bernardi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
use super::tank::Tank;
use csv;
use std::fs::File;
pub(super) fn save_tank_report(path: &str, tanks: &Vec<Tank>) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut wtr = csv::Writer::from_writer(file);
    wtr.write_record(["Name", "Damage", "Energy"])?;
    for tank in tanks {
        let damage = &format!("{}", tank.damage);
        let energy = &format!("{}", tank.energy);
        let name: &str = &tank.name;
        wtr.write_record([name, damage, energy])?;
    }
    wtr.flush()?;
    Ok(())
}
