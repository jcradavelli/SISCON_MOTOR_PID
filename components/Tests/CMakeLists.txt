
if(NOT DEFINED IDF_PATH)
cmake_minimum_required(VERSION 3.13)  # CMake version check
project(TEST_PID)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard


# # Include headers
# get_filename_component(PARENT_DIR ../ ABSOLUTE)
# message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# message(PARENT_DIR="${PARENT_DIR}")

# # Add main.cpp file of the project root directory as a source file
# set(SOURCE_FILES main_test_PID.c)

# set(PID_LIB_DIR     "${PARENT_DIR}/PIDcontroller")

add_definitions(-DPC_BUILD)

set(SOURCE_FILES    "main_test_PID.c"
                    )


# Add executable target with source files listed in SOURCE_FILES variable
add_executable(TEST_PID ${SOURCE_FILES})


# add_custom_target(runn ALL)

add_custom_command(
    TARGET "TEST_PID" POST_BUILD
    COMMAND powershell "Debug/TEST_PID.exe | tee TestResul.txt"
    DEPENDS "Debug/TEST_PID.exe"
    COMMENT "Executando Testes..." 
    USES_TERMINAL
    VERBATIM
)


endif()
