#!/bin/bash

rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=1.01 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=0.9 _v_target:=0.0 _w_target:=1.5
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=1.01 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=0.9 _v_target:=0.0 _w_target:=1.5
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=1.01 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=0.9 _v_target:=0.0 _w_target:=1.5
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=1.01 _w_target:=0.0
