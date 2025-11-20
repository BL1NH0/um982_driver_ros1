cmake_minimum_required(VERSION 3.0.2)
project(um982_driver)

## Encuentra los paquetes de ROS necesarios
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  nmea_msgs
  rospy
  sensor_msgs
  std_msgs
)

## Configuración del paquete catkin
catkin_package(
  CATKIN_DEPENDS nav_msgs nmea_msgs rospy sensor_msgs std_msgs
)

## Directorios de includes
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Instala los scripts Python como ejecutables
catkin_install_python(PROGRAMS
  scripts/um982_serial_driver.py
  scripts/um982_topic_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Instala los scripts de Bash como ejecutables
install(PROGRAMS
  sh/str2str_ntrip.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Instala los archivos launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Instala otros archivos de configuración si existen
install(DIRECTORY sh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sh
  FILES_MATCHING PATTERN "*.sh"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)
