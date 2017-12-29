#!/bin/bash

#scp -r ~/gopigo_ws roman@192.168.0.105:/home/roman/gopigo_ws
rsync -avr -e "ssh -l roman" --exclude 'build' --exclude 'devel' ~/gopigo_ws* 192.168.0.105:/home/roman/
