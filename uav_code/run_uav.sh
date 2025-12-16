#!/usr/bin/env bash

nohup python3 detect.py > detect.log 2>&1 & 
sleep 1
nohup python3 main.py > main.log 2>&1 &
