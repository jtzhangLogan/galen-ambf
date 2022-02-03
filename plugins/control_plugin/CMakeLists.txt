set(HDR_FILES GalenControlPlugin.h GalenInterface.h GalenSimpModel.h GalenInvKin.h)
set(SRC_FILES GalenControlPlugin.cpp GalenInterface.cpp GalenSimpModel.cpp GalenInvKin.cpp)

add_library(GalenControlPlugin SHARED ${HDR_FILES} ${SRC_FILES})

target_link_libraries (GalenControlPlugin ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ambf_framework chai3d)

# TODO: Whats this position code?
set_property(TARGET GalenControlPlugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)