project(Photino)
cmake_minimum_required(VERSION 3.5)

set(PROJECT_SOURCE_DIR     ${CMAKE_SOURCE_DIR}/src)
set(CMAKE_BINARY_DIR       ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH    ${CMAKE_BINARY_DIR})

set(StdFeatures
	cxx_range_for
	cxx_noexcept
	cxx_rvalue_references
	cxx_override
	cxx_final
	cxx_deleted_functions
	cxx_delegating_constructors
	cxx_auto_type
	cxx_constexpr
	)

# Enable threading
find_package(Threads REQUIRED)

# Do not remove the following auto-generated signatures

# Auto-generated. Do not edit. All changes will be undone
set(SourceFiles
    ${PROJECT_SOURCE_DIR}/main.cpp
   )
# Auto-generated end


add_executable(Photino ${SourceFiles})
target_link_libraries(Photino ${CMAKE_THREAD_LIBS_INIT})
