cmake_minimum_required(VERSION 3.8)
project(app)

########
# test #
########

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_m2p test/test_m2p.py)
endif()