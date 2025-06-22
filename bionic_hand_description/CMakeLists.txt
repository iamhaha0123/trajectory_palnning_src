cmake_minimum_required(VERSION 3.8)
project(bionic_hand_description)

find_package(ament_cmake REQUIRED)

# 把所有靜態資料夾安裝到 share/${PROJECT_NAME}
install(DIRECTORY launch meshes urdf world config
        DESTINATION share/${PROJECT_NAME})

ament_package()
