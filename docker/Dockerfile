# Official ROS2 Humble image with desktop support
FROM osrf/ros:humble-desktop

# Python and system dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    git \
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \
    sudo && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install git+https://github.com/DLu/tf_transformations.git

COPY requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt

# Create non-root user for development
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Improve shell prompt and history behavior
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc && \
    echo 'HISTFILE=~/.bash_history' >> ~/.bashrc && \
    echo 'HISTSIZE=1000' >> ~/.bashrc && \
    echo 'HISTFILESIZE=2000' >> ~/.bashrc && \
    echo 'shopt -s histappend' >> ~/.bashrc && \
    echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> ~/.bashrc

# Configure ROS2 and Cyclone DDS environment

COPY cyclonedds.xml /etc/cyclonedds.xml
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /home/$USERNAME/workspace/install/setup.bash ] && source /home/$USERNAME/workspace/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///etc/cyclonedds.xml" >> ~/.bashrc

#Setup workspace
COPY setup.sh /setup.sh
COPY entrypoint.sh /home/devuser/workspace/entrypoint.sh

USER root 
RUN chmod +x /setup.sh
RUN echo "source /setup.sh" >> /home/$USERNAME/.bashrc
RUN chmod +x /home/devuser/workspace/entrypoint.sh

COPY src /home/$USERNAME/workspace/src
USER root
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/workspace

USER $USERNAME
WORKDIR /home/$USERNAME/workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN echo "source /home/$USERNAME/workspace/install/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/workspace/src
ENTRYPOINT ["/home/devuser/workspace/entrypoint.sh"]
CMD ["ros2", "run", "rmv_chore", "rmv_chore"]
