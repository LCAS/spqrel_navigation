#!/bin/bash
xterm -hold -e "ssh nao@$PEPPER_IP < launch_d2l.sh" &
sleep 1
xterm -hold -e "ssh nao@$PEPPER_IP < launch_localizer.sh" &
sleep 1
xterm -hold -e "ssh nao@$PEPPER_IP < launch_planner.sh" 
