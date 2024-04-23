
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(policy_ctrl main.cpp policy_ctrl.cpp)
target_link_libraries(policy_ctrl robot biomimetics dynacore_param_handler qpOASES)

# Create the subscriber executable
