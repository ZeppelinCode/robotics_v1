#!/bin/bash

. install/setup.bash
#ros2 launch robo_miner_controller launch.py
ros2 run --prefix 'gdbserver localhost:3000' robo_miner_controller robo_miner_controller
#ros2 run --perfix 'gdb -ex run --args' robo_miner_controller robo_miner_controller
