cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_msgs_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_DIR}/msg")
  message(STATUS "Initializing ${COMMON_DIR} submodule. This should only happen once.")

  # Make sure we can find the git executable
  find_package(Git)
  if(NOT Git_FOUND)
    message(FATAL_ERROR "Unable to initialize submodule because we could not find the git executable. Please clone this repo using 'git clone --recursive'")
  endif()

  # Initialize and update the submodule
  execute_process(
    WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} submodule update --recursive --init
  )
endif()

# Generate Service Files
set(SRV_DIR "${COMMON_DIR}/srv")
file(GLOB SRV_FILES RELATIVE "${SRV_DIR}"
    "${SRV_DIR}/*.srv")
set(MIP_SRV_DIR "${SRV_DIR}/mip")
file(GLOB MIP_SRV_FILES RELATIVE "${MIP_SRV_DIR}"
    "${MIP_SRV_DIR}/*.srv")
add_service_files(
  DIRECTORY ${SRV_DIR}
  FILES ${SRV_FILES}
)
add_service_files(
  DIRECTORY ${MIP_SRV_DIR}
  FILES ${MIP_SRV_FILES}
)

# Generate Message Files
set(MSG_DIR "${COMMON_DIR}/msg")
file(GLOB MSG_FILES RELATIVE "${MSG_DIR}"
    "${MSG_DIR}/*.msg")
set(MIP_MSG_DIR "${MSG_DIR}/mip")
file(GLOB MIP_MSG_FILES RELATIVE "${MIP_MSG_DIR}"
    "${MIP_MSG_DIR}/*.msg")
set(MIP_STRUCT_MSG_DIR "${MIP_MSG_DIR}/struct")
file(GLOB MIP_STRUCT_MSG_FILES RELATIVE "${MIP_STRUCT_MSG_DIR}"
    "${MIP_STRUCT_MSG_DIR}/*.msg")
set(MIP_BITFIELD_MSG_DIR "${MIP_MSG_DIR}/bitfield")
file(GLOB MIP_BITFIELD_MSG_FILES RELATIVE "${MIP_BITFIELD_MSG_DIR}"
    "${MIP_BITFIELD_MSG_DIR}/*.msg")
add_message_files(
  DIRECTORY ${MSG_DIR}
  FILES ${MSG_FILES}
)
add_message_files(
  DIRECTORY ${MIP_MSG_DIR}
  FILES ${MIP_MSG_FILES}
)
add_message_files(
  DIRECTORY ${MIP_STRUCT_MSG_DIR}
  FILES ${MIP_STRUCT_MSG_FILES}
)
add_message_files(
  DIRECTORY ${MIP_BITFIELD_MSG_DIR}
  FILES ${MIP_BITFIELD_MSG_FILES}
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
