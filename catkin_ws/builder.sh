#!/bin/sh

WATCH_DIRS=(~/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/src)

inotifywait -m ${WATCH_DIRS[@]} -e close_write |
    while read path action file; do
        if [[ "$file" =~ .*cpp$ ]] || [[ "$file" =~ .*h$ ]]; then
            echo "$file modified, rebuilding..."
            if catkin --force-color build | grep "succeeded!"; then
                echo "Enter roslaunch in new terminal next." 
            fi
        fi
    done
