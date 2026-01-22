FROM robopaas/rap-jazzy:cuda12.5.0

RUN sudo apt update
RUN sudo apt install ros-jazzy-tf2-ros

RUN git clone https://github.com/rfsjsu/RAP-2025-Project-Group-2.git ~/rap/Gruppe2
ADD api-key.txt /home/ros/rap/Gruppe2/api-key.txt

RUN pip3 install jpl-rosa --break-system-packages
RUN pip3 install anthropic --break-system-packages
RUN pip3 install langchain-anthropic==0.3.22 --break-system-packages

RUN bash -c "source ~/rap/Gruppe2/init.sh"
ENV GZ_SIM_RESOURCE_PATH=/home/ros/rap/Gruppe2/world/models

# Build workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ~/colcon_ws && colcon build"

# Create entrypoint script
RUN sudo bash -c "touch /ros_entrypoint.sh && chown $USER /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh"
RUN cat > /ros_entrypoint.sh <<'EOF'
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/ros/colcon_ws/install/setup.bash"

exec "$@"
EOF

ENTRYPOINT ["/ros_entrypoint.sh"]
