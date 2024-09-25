
FROM osrf/ros:humble-desktop
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
 
# Создание рабочей области ROS 
RUN mkdir /photobot_ws
COPY photobot_ws/ /photobot_ws
WORKDIR /photobot_ws

# Установка необходимых пакетов в систему, включая пакеты для поддержки лидаров и GPS-приемников
RUN source /opt/ros/humble/setup.bash \
 && apt-get update -y \
 && apt-get install -y openssh-server supervisor git ros-humble-sick-scan-xd ros-humble-novatel-gps-driver ros-humble-camera-info-manager \
 && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
 && colcon build --symlink-install

# Установка пакетов ROS 2

# USB-камера
WORKDIR /photobot_ws/src
RUN git clone https://github.com/klintan/ros2_usb_camera.git
WORKDIR /photobot_ws
RUN . /opt/ros/humble/setup.bash && colcon build

# IMU-сенсор
WORKDIR /photobot_ws/src
RUN git clone https://github.com/DEMCON/ros2_xsens_mti_driver.git
WORKDIR /photobot_ws
RUN . /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON

# Контроллеры двигателей колес
WORKDIR /photobot_ws/src
RUN git clone https://github.com/odriverobotics/ros_odrive.git
WORKDIR /photobot_ws
RUN . /opt/ros/humble/setup.bash && colcon build --packages-select odrive_can

# Настройка доступа по SSH
RUN echo 'root:root' | chpasswd
RUN echo "PermitRootLogin yes" >> "/etc/ssh/sshd_config"
RUN service ssh start 
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /photobot_ws/install/setup.bash" >> /root/.bashrc

# Настройка процессов автозапуска
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY ros_entrypoint.sh /
RUN chmod a+x /ros_entrypoint.sh
EXPOSE 22
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["/usr/bin/supervisord"]
