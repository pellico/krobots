# This tank move to center and then stop
# it tries to scan fire to all tanks it detects
import ktanks,argparse
from math import pi,cos

def angle_wrapping(angle:float) -> float :
    """
    Wrap angle in the range ]pi,pi]
    """
    angle_res = angle
    while True: 
        if angle_res > pi :
            angle_res=angle_res - 2.0 * pi
        elif angle_res <= -pi:
            angle_res=angle_res + 2.0 * pi
        else:
            break
    
    return angle_res



def main_loop(name,ip,port):
    tank=ktanks.Tank(name,ip,port) # Create the proxy object to communicate with server
    stop_distance=150.0 # Below this distance from center tank shall stop and just search and fire target.
    status = tank.get_status()
    forward_power = 0.9   # Initial tank body forward movement power
    tank.set_engine_power(forward_power,0.0)
    # Angle of power source from tank position in world coordinates. (not referred to tank direction)
    target_angle = status.power_source.p

    while True :
        # Radar manager and fire control
        radar_result = tank.get_radar_result(-0.17,0.17)
        #Keep aligned radar and cannon
        tank.set_cannon_position(radar_result.angle)  
        # If some tank detect fire to them.
        if radar_result.tanks:
            tank.set_cannon_position(tank.get_radar_result(-0.17,0.01).angle)
            # Set smaller radar width to increase accuracy when tank find enemy
            # so it increase accuracy of angle for more accurate firing
            for _ in range(0,34):
                radar_result = tank.get_radar_result(0.01,0.01)
                tank.set_cannon_position(radar_result.angle)
                if radar_result.tanks and radar_result.tanks[0].distance < tank.simulation_configuration.bullet_max_range:
                    tank.fire_cannon()
                     
        
        # When passing the distance limit stop_distance.
        # slow down the tank before changing direction. 
        if status.power_source.r <  stop_distance:
            if forward_power != 0.0: 
                # Slow down for easier turning
                while True:
                    status = tank.get_status()
                    # this is speed in the direction of tank.
                    vel_direction = status.velocity.r * cos(status.velocity.p - status.angle)
                    if vel_direction < 0.1:
                        break
                    tank.set_engine_power(-0.001*vel_direction,0)
                forward_power=0.0
                tank.set_engine_power(0,0)
        else:
            forward_power = 0.9
            target_angle = status.power_source.p
            # Adjust tank angle to keep correct direction    
            status = tank.get_status()
            delta_ang = angle_wrapping(target_angle- status.angle)
            angimp_set = (0.5*delta_ang - 0.01*status.angvel)*abs(status.angvel)
            # If too big error angle reduce forward speed 
            tank.set_engine_power(forward_power/(1+10*abs(delta_ang)),angimp_set) 
            


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Dumb tank")
    parser.add_argument("name", type=str, help="Name of tank max 20 chars")
    parser.add_argument(
        "--ip",
        required=False,
        type=str,
        help="server ip address",
        default = "127.0.0.1"
    )
    
    parser.add_argument(
        "--port",
        required=False,
        type=int,
        help="server port number",
        default = 55230
    )
    args = parser.parse_args()
    main_loop(args.name,args.ip,args.port)