cmake_minimum_required(VERSION 2.8.3)
project(ros_rls_estimation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cisst_msgs
  cisst_ros_bridge
  roscpp
  std_msgs
  cmake_modules
  dynamic_reconfigure
)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstParameterTypes
     )

# find cisst and make sure required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

generate_dynamic_reconfigure_options(
    cfg/RLS.cfg
    )
 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES force_estimation
#  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
 if( cisst_FOUND)

 include (${CISST_USE_FILE})


add_executable( 
      ros_rls_estimation 
      src/main.cpp )

 add_dependencies(ros_rls_estimation ${PROJECT_NAME}_gencfg)

  target_link_libraries( ros_rls_estimation ${catkin_LIBRARIES})
  cisst_target_link_libraries( ros_rls_estimation ${REQUIRED_CISST_LIBRARIES})
  
endif( cisst_FOUND )



