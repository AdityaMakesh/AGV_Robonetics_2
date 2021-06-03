cmake_minimum_required(VERSION 2.8.3)
project(rabbit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sereal
  std_msgs
  tf
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

## Generate messages in the 'msg' folder
add_service_files(
FILES
autoCalibrate.srv
getAbsolutePostion.srv
getFeedbackGain.srv
getIntegralGain.srv
getMaxMotorSpeed.srv
getPositionEncoder.srv
getProportionateGain.srv
getRelativePostion.srv
loadFactorySettings.srv
readDamping.srv
readMotorSpeed.srv
setAbsolutePostion.srv
setFeedbackGain.srv
setIntegralGain.srv
setMaxMotorSpeed.srv
setMotorSpeed.srv
setPositionEncoder.srv
setProportionateGain.srv
setRelativePostion.srv
writeDamping.srv
	)

add_message_files(DIRECTORY msg FILES pose.msg)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   
   std_msgs
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rabbit
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp sereal std_msgs tf
#  DEPENDS system_lib
)

## Build ##
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(rabbit src/ServoMotor.cpp)
target_link_libraries(rabbit ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## Declare a C++ executable
add_executable(rnode src/Rhino_Node.cpp src/ServoMotor.cpp)
add_dependencies(rnode rabbit_generate_messages_cpp)
target_link_libraries(rnode  ${catkin_LIBRARIES} sereal)

add_executable(pnode src/PSuedoRhino_Node.cpp src/PsuedoServoMotor.cpp)
add_dependencies(pnode rabbit_generate_messages_cpp)
target_link_libraries(pnode  ${catkin_LIBRARIES} sereal)

install(DIRECTORY include/rabbit/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
