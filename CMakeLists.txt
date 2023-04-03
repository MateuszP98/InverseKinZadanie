cmake_minimum_required(VERSION 3.24)
project(InverseKinZadanie)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(InverseKinZadanie
        InverseKinematics_2DOF.cpp
        InverseKinematics_2DOF.h
        InverseKinZadanie.cpp)
