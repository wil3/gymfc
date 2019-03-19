This directory contains a collection of scripts you can use to test to make sure GymFC is
interacting correctly with the aircraft model before integration of the
extended environment and controller. 


## test_start_sim.py
Minimalist example (e.g. hello world), loads all plugins and model.

## test_step_sim.py
Apply control signals from the command line, ensure motor rotations are
correct.

Example use,
```
python3 test_step_sim.py <path to aircraft json config file>  0 1 1 0 --delay 0.1
```
