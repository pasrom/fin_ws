#!/bin/bash

rsync -avr -e "ssh -l roman" --exclude 'build' --exclude 'devel' ~/fin_ws* 192.168.0.105:/home/roman/
