This directory contains a collection of scripts you can use to test to make sure GymFC is
interacting correctly with the aircraft model before integration of the
extended environment and controller. 

Execute these in order.

1) **test_start_sim.py**

Simplest test (e.g., hello world). Confirm plugins and model will load.

Example use,
```
python3 test_start_sim.py <path to aircraft model SDF>
```
or more explicitly, 
```
python3 test_start_sim.py ../modules/digitaltwins/nf1/model.sdf
```

The aircraft model should be present in the middle of the world if successful. 

2) **check_sim_stability.py**

(Optional: Skip if using DART) Used to check the stability of the model in the simulation. This will 'excite'
the aircraft by ramping each permutation of the control signals to achieve
maximum angular velocities of the aircraft and measure the distance between all of the aircraft model components. 
The script will
output a plot indicating the total distance the model shifted during the
experiment.  

Note: Requires [py3gazebo](https://github.com/wil3/py3gazebo) and Matplotlib
Example use,
```
python3 check_sim_stability.py <path to aircraft model SDF> --max-sim-time=1 --gazebo-version=11
```


3) **test_step_sim.py**
Ensure motor rotation are correct and motor numberings are correct by apply control signals from the command line, 

Example use,
```
python3 test_step_sim.py <path to aircraft model SDF>  0 1 1 0 --delay 0.1
```

4) **test_axis.py**

Test IMU is working properly and oriented correctly.

Example use,
```
python3 test_axis.py <path to aircraft model SDF>
```
