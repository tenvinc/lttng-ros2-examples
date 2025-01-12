FROM osrf/ros:humble-desktop-full-jammy AS base

ARG HOST_UID=1000
ARG HOST_GID=1000

RUN apt update && apt install -y sudo

# Setup user
RUN groupadd -g ${HOST_GID} tracer
RUN useradd -m -d /home/tracer -u ${HOST_UID} -g ${HOST_GID} tracer
RUN adduser tracer sudo
RUN passwd -d tracer

USER tracer
WORKDIR /home/tracer
RUN mkdir -p /home/tracer/trace_ws/src

# Setup tracing
WORKDIR /home/tracer/trace_ws
USER root
RUN apt update && apt install -y lttng-tools liblttng-ust-dev python3-lttng
USER tracer
RUN git clone -b 4.1.1 https://github.com/ros2/ros2_tracing.git src/ros2_tracing
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-up-to tracetools"

WORKDIR /home/tracer/trace_ws
USER tracer
RUN echo "source /opt/ros/humble/setup.bash" >> /home/tracer/.bashrc
RUN echo "source /home/tracer/trace_ws/install/setup.bash" >> /home/tracer/.bashrc
