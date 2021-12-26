#!/usr/local/bin/python2.7
# encoding: utf-8
'''
multi_launcher -- Easier launch multi instance of tank

multi_launcher is a convenience tool to launch multiple isntance of same tank

@author:     Oreste Bernardi

@copyright:  2021 organization_name. All rights reserved.

@license:    GPL v3

@contact:    
@deffield    updated: Updated
'''
import argparse,pathlib
from subprocess import PIPE,Popen

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Tank launcher")
    parser.add_argument(
        "tank_script",
        type=pathlib.Path,
        help="Path to tank python script"
        )
    parser.add_argument(
        "num_tanks",
        type=int,
        help="Num anks to launch"
        )
    parser.add_argument(
        "--name", 
        type=str,
        required = False, 
        help="Base name of tank max 20 chars",
        default = "tank"
        )
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
    print(args.num_tanks)
    for index in range(args.num_tanks):
        command = 'python ' + str(args.tank_script) + f" {args.name}_{index} --ip {args.ip} --port {args.port}"
        print(command)
        # run(
        #     ["dir"],
        #     shell = True,
        #     stdout=PIPE
        #     )
        Popen(
            ["python.exe",str(args.tank_script),args.name+"_"+f"{index}"],
            shell = True,
            stdout=PIPE
            )
        print("Pippo")
        