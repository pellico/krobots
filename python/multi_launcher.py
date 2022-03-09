#!python
# encoding: utf-8
'''
multi_launcher -- Easier launch multi instance of tank

multi_launcher is a convenience tool to launch multiple tanks

@author:     Oreste Bernardi

@copyright:  2021 organization_name. All rights reserved.

@license:    GPL v3

@contact:    
@deffield    updated: Updated
'''
import argparse, pathlib, time, os, random
from subprocess import PIPE, Popen

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Launch all tank clients available in a folder")
    parser.add_argument(
        "tanks_folder",
        type=pathlib.Path,
        help="Path to tank python script folder"
        )
    parser.add_argument(
        "--ip",
        required=False,
        type=str,
        help="server ip address",
        default="127.0.0.1"
    )
    
    parser.add_argument(
        "--port",
        required=False,
        type=int,
        help="server port number",
        default=55230
    )
    parser.add_argument(
        "--random_seed",
        required=False,
        type=int,
        help="seed for pseudo randomic tank order selection",
        default=0
    )
    args = parser.parse_args()
    clients = sorted(map(lambda x: x.path, filter(lambda x: x.is_file(), os.scandir(args.tanks_folder))))
    random.seed(args.random_seed)
    random.shuffle(clients)
    procs = []
    for client_path in clients:
        command = 'python ' + str(client_path) + f" --ip {args.ip} --port {args.port}"
        print(command)
        if os.name == "nt":
            # I need shell true in order to use the virtualenv
            shell = True
        else:
            # in Linux shell true has issue and it seems that it is not required.
            shell = False
        procs.append(Popen(
            ["python", str(client_path), "--ip", str(args.ip), "--port", str(args.port)],
            shell=shell,
            stdout=PIPE,))
        time.sleep(1)
    for x in procs:
        while x.poll() == None:
            time.sleep(1)
        
