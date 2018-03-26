cmake_minimum_required(VERSION 2.8.3)
project(run_script)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/run.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
