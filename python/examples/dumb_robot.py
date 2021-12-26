import ktanks,argparse
from time import sleep
from math import radians,pi,sqrt,cos


def angle_wrapping(angle:float) -> float :
    """
    Wrap angle in the range ]pi,pi]
    """
    angle_res = angle;
    while True: 
        if angle_res > pi :
            angle_res=angle_res - 2.0 * pi
        elif angle_res <= -pi:
            angle_res=angle_res + 2.0 * pi
        else:
            break
    
    return angle_res



def main_loop(name,ip,port):
    tank=ktanks.Tank(name,ip,port)
    turn_back_distance=150.0 # At this distance from power source turn back to power source
    status = tank.get_status()
    forward_power = 0.05    # Initial tank body power
    tank.set_engine_power(forward_power,0.0)
    target_angle = status.power_source.p
    last_power_distance = tank.get_status().power_source.r
    print(tank.simulation_configuration)     
    while True :
        # Radar manager and fire control
        radar_result = tank.get_radar_result(-0.17,0.17)
        tank.set_cannon_position(radar_result.angle)  #Keep aligned radar and cannon
        # If some tank detect fire to them.
        if radar_result.tanks:
            tank.set_cannon_position(tank.get_radar_result(-0.17,0.01).angle)
            # Set smaller radar width to increase accuracy.
            for _ in range(0,34):
                radar_result = tank.get_radar_result(0.01,0.01)
                tank.set_cannon_position(radar_result.angle)
                if radar_result.tanks:
                    tank.fire_cannon()
        
        # Adjust tank angle to keep heading    
        status = tank.get_status()
        delta_ang = angle_wrapping(target_angle- status.angle)
        angimp_set = (0.05*delta_ang - 0.02*status.angvel)*abs(status.angvel)
        # If too big error angle reduce forward speed 
        tank.set_engine_power(forward_power/(1+10*abs(delta_ang)),angimp_set)
        
        # When passing the boundary we set by  turn_back_distance 
        # turn back to power source
        if status.power_source.r > turn_back_distance and last_power_distance < turn_back_distance:
            # Slow down for easier turning
            while True:
                status = tank.get_status()
                # this is speed in the direction of tank.
                vel_direction = status.velocity.r * cos(status.velocity.p - status.angle)
                if vel_direction < 0.01:
                    break
                tank.set_engine_power(-0.001*vel_direction,0)

            #update target angle to the direction of power source
            target_angle = status.power_source.p
            
        last_power_distance = status.power_source.r
        
        



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
