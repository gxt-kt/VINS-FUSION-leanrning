#!/bin/bash

source ./devel/setup.bash  
nohup bash /media/home/gxt_kt/docker/clion/clion-2023.2.1/bin/clion.sh \
  `pwd`/src/VINS-Fusion-DetailedNote/vins_estimator&>/dev/null &
