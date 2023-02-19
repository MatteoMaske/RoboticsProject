#!/bin/bash

gnome-terminal --tab --title="planner" --command="bash -c 'rosrun cpp_publisher planner;$SHELL'"
gnome-terminal --tab --title="move" --command="bash -c 'rosrun cpp_publisher move;$SHELL'"
gnome-terminal --tab --title="vision" --command="bash -c 'rosrun py_publisher vision.py;$SHELL'"
