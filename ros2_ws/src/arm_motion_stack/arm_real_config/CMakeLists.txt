cmake_minimum_required(VERSION 3.10)
project(arm_real_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_real_arm_profile test/test_real_arm_profile.py)
  ament_add_pytest_test(
    test_remote_deployment_contract
    test/test_remote_deployment_contract.py
  )
endif()

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
