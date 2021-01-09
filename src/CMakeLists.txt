# CMakeList.txt : CMake project for AeroOptimizer, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (AeroOptimizer "../src/AeroOptimizer.cpp" "../src/AeroOptimizer.h" "Optimization/NewtonsMethod.h" "Optimization/NewtonsMethod.cpp" "LinearAlgebra.h" "LinearAlgebra.cpp" "Aerodynamics/PointMass.h" "Typedefs.h" "Aerodynamics/Aerofoil.h" "Aerodynamics/Wing.h" "Aerodynamics/Aircraft.h" "Aerodynamics/Aircraft.cpp" "Aerodynamics/Wing.cpp")

# TODO: Add tests and install targets if needed.
