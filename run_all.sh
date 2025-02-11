#!/bin/bash

# Run roscore in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && roscore; exec bash"

# Run qibullet_ros_node2.py in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && cd $(pwd)/qibullet_scripts && python3 qibullet_ros_node2.py; exec bash"

# Run tts_node.py in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && cd $(pwd)/qibullet_scripts && python3 tts_node.py; exec bash"

# Run face_detection.py in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && cd $(pwd)/qibullet_scripts && python3 face_detection.py; exec bash"

# Run Rasa actions in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && cd $(pwd)/rasa && python3 -m rasa_sdk --actions actions; exec bash"

# Run Rasa shell in a new terminal
gnome-terminal -- bash -c "source ~/.bashrc && qib && cd $(pwd)/rasa && rasa shell; exec bash"
