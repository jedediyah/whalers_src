#!/bin/bash 

source /opt/nasa/indigo/setup.bash
killall gzserver > /dev/null 2>&1
killall rosout > /dev/null 2>&1
killall roslaunch > /dev/null 2>&1
killall rosmaster > /dev/null 2>&1
killall roscore > /dev/null 2>&1
killall python > /dev/null 2>&1
killall python > /dev/null 2>&1

while true 
do

    echo "Launching simulation..."
    roslaunch srcsim qual2.launch extra_gazebo_args:="-r" init:="true" > /dev/null 2>&1 &
    sleep 1m
    python ~/whalers_src/qual2/experiment.py &
    python ~/whalers_src/qual2/calculateScore.py &

    echo "Sleeping for 3 minutes..."
    sleep 3m

    echo "Killing everything!" 
    killall gzserver > /dev/null 2>&1
    killall rosout > /dev/null 2>&1
    killall roslaunch > /dev/null 2>&1
    killall rosmaster > /dev/null 2>&1
    killall roscore > /dev/null 2>&1
    killall python > /dev/null 2>&1
    killall python > /dev/null 2>&1
    sleep 45s
    killall gzserver > /dev/null 2>&1
    killall rosout > /dev/null 2>&1
    killall roslaunch > /dev/null 2>&1
    killall rosmaster > /dev/null 2>&1
    killall roscore > /dev/null 2>&1
    killall python > /dev/null 2>&1
    killall python > /dev/null 2>&1
    sleep 45s
done



