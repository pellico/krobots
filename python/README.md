# Python package to communicate with krobot server

## How to install
* Download ktanks-x.x.x.tar.gz
* Install it using your preferred package manager

```bash
pip install ktanks-x.x.x.tar.gz
```

## Documentation

ktanks documentation is available at this link:
[Python package Documentation](https://pellico.github.io/krobots/)


## How to run example **dumb_robot**
In `examples` is available `dumb_robot.py` and `tower.py`.

This robot move back and forward inside the zero power limit circle while looking for enemy and firing if it found an enemy. This tank try always to pass through the power source.
In one shell execute the game server waiting for 2 tanks

```bash
krobots 2 
```

in another two shells launch  dumb_robot.py

```bash
python dumb_robot.py
```

You can launch dumb robot specifying server port and ip address see the help for detailed option description.

```bash
python dumb_robot.py --help
``` 

## Multi tank launcher

If you want to launch automatically many tanks it is possible to use the helper script `multi_launcher`

### Example
```bash
python multi_launcher.py .\examples
```
This script expect tank script is supporting the same command line options as `dumb_robot.py`
For other options of `multi_launcher.py` look at command line help.

```bash
python multi_launcher.py --help
```