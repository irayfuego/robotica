cmake_minimum_required(VERSION 3.8)
project(robotica_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Mensajes custom (añade aquí los .msg que vayas creando)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/MotorCommand.msg"
  "msg/ServoCommand.msg"
  DEPENDENCIES std_msgs
)

ament_package()
