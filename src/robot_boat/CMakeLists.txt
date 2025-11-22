cmake_minimum_required(VERSION 3.8)
project(robot_boat)

# find dependencies
find_package(ament_cmake REQUIRED)

# Dependencias para construcción (solo las necesarias para build)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)

# Instalar archivos de lanzamiento
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos URDF
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos de mallas (meshes)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos de configuración
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
