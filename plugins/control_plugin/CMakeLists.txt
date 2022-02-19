set(HDR_FILES GalenControlPlugin.h GalenInterface.h GalenSimpModel.h GalenInvKin.h)
set(SRC_FILES GalenControlPlugin.cpp GalenInterface.cpp GalenSimpModel.cpp GalenInvKin.cpp)

add_subdirectory(plugin_msgs)
find_package(catkin COMPONENTS plugin_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_library(GalenControlPlugin SHARED ${HDR_FILES} ${SRC_FILES})
add_dependencies(GalenControlPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries (GalenControlPlugin ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${catkin_LIBRARIES} ambf_framework chai3d)

# TODO: Whats this position code?
set_property(TARGET GalenControlPlugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)
