# CMakeList.txt : CMake project for AeroOptimizer, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (AeroOptimizer "../src/AeroOptimizer.cpp" "../src/AeroOptimizer.h" "NewtonsMethod.h" "NewtonsMethod.cpp" "LinearAlgebra.h" "LinearAlgebra.cpp")

# TODO: Add tests and install targets if needed.
