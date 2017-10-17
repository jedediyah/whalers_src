#!/bin/bash

source /opt/nasa/indigo/setup.bash 

for dir in ./2017*
do
    echo " " 
    echo $dir
    cd $dir/gzserver
    
    FILE="qual_2.log"
    if [ -f $FILE ]; then
       #echo "   File '$FILE' Exists.  Scoring..."
       cd ~/srcsim_score
       ./scoring_q2.rb ~/.gazebo/log/$dir/gzserver/qual_2.log
    else
       gz log -e -f state.log --filter *.pose/*.pose -z 60 > qual_2.log
    fi
    cd ~/.gazebo/log
    echo " "
    
done

