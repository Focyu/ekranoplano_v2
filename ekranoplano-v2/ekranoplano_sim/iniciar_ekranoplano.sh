#!/bin/bash
echo "🚀 Iniciando simulación del Ekranoplano en Gazebo (ROS 2 Humble)..."

source /opt/ros/humble/setup.bash
cd ~/Documentos/ekranoplano_sim
export GAZEBO_MODEL_PATH=${PWD}/models:$GAZEBO_MODEL_PATH

# Lanzar el publicador de olas en segundo plano
echo "🌊 Iniciando publicador de olas (Sea State 2)..."
python3 ${PWD}/ola_publisher.py &
OLA_PID=$!
echo "   ola_publisher PID: $OLA_PID"

# Lanzar Gazebo
ros2 launch gazebo_ros gazebo.launch.py world:=vuelo.world

# Al cerrar Gazebo, matar también el nodo de olas
kill $OLA_PID 2>/dev/null
echo "✅ Simulación terminada."
