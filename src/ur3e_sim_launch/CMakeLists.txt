cmake_minimum_required(VERSION 3.8)
project(ur3e_sim_launch)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Install Python launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                   GROUP_READ GROUP_EXECUTE
                   WORLD_READ WORLD_EXECUTE
)

# Install config files if you have any
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resources if you have any
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()

