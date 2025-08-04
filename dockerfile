FROM osrf/ros:humble-desktop

ENV ROS_DOMAIN_ID=1

# Instala WebSocket++ e nav-msgs
WORKDIR /ros2_ws
RUN apt-get update && apt-get install -y \
    libwebsocketpp-dev \
    ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copia o pacote para o workspace
WORKDIR /ros2_ws/src
COPY ./src/ ./

# rosdep
WORKDIR /ros2_ws
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
    
# Compila o pacote
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --merge-install"

# Expoe a porta do WebSocket
EXPOSE 8080

# Comando padrao para iniciar o no'
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && bash"]