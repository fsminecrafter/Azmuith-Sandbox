cmake_minimum_required(VERSION 3.24)
project(AzmuithSandbox VERSION 0.1.0 LANGUAGES C CXX)

include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    set(AZMUITH_GPU_DEFAULT ON)
else()
    set(AZMUITH_GPU_DEFAULT OFF)
endif()
option(AZMUITH_ENABLE_PHYSX_GPU "Build PhysX GPU dynamics with CUDA" ${AZMUITH_GPU_DEFAULT})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Vulkan 1.1 REQUIRED)
find_package(Threads REQUIRED)

include(FetchContent)

FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE)
FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.9
    GIT_SHALLOW TRUE)
FetchContent_Declare(physx
    GIT_REPOSITORY https://github.com/NVIDIA-Omniverse/PhysX.git
    GIT_TAG 107.3-physx-5.6.1
    GIT_SHALLOW TRUE)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
set(PX_BUILDSNIPPETS OFF CACHE BOOL "" FORCE)
set(PX_BUILDPVDRUNTIME OFF CACHE BOOL "" FORCE)
set(PX_GENERATE_GPU_PROJECTS ${AZMUITH_ENABLE_PHYSX_GPU} CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(glfw glm nlohmann_json)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()
FetchContent_GetProperties(physx)
if(NOT physx_POPULATED)
    FetchContent_Populate(physx)
endif()

set(PHYSX_ROOT_DIR "${physx_SOURCE_DIR}/physx" CACHE PATH "PhysX SDK source root" FORCE)
if(NOT DEFINED TARGET_BUILD_PLATFORM)
    if(WIN32)
        set(TARGET_BUILD_PLATFORM "windows" CACHE STRING "PhysX target platform")
    elseif(UNIX AND NOT APPLE)
        set(TARGET_BUILD_PLATFORM "linux" CACHE STRING "PhysX target platform")
    endif()
endif()
add_subdirectory("${PHYSX_ROOT_DIR}/compiler/public" "${physx_BINARY_DIR}/public")

add_library(imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp")
target_include_directories(imgui PUBLIC "${imgui_SOURCE_DIR}" "${imgui_SOURCE_DIR}/backends")
target_link_libraries(imgui PUBLIC glfw Vulkan::Vulkan)

add_executable(azmuith_sandbox src/main.cpp)
target_include_directories(azmuith_sandbox PRIVATE "${PHYSX_ROOT_DIR}/include")
target_link_libraries(azmuith_sandbox PRIVATE
    Vulkan::Vulkan
    glfw
    glm::glm
    nlohmann_json::nlohmann_json
    imgui
    PhysX
    PhysXCommon
    PhysXFoundation
    PhysXExtensions
    Threads::Threads)
target_compile_definitions(azmuith_sandbox PRIVATE GLM_FORCE_DEPTH_ZERO_TO_ONE)
find_program(GLSLC_EXECUTABLE NAMES glslc HINTS "$ENV{VULKAN_SDK}/bin" "$ENV{VULKAN_SDK}/Bin")
if(GLSLC_EXECUTABLE)
    set(AZMUITH_SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
    file(MAKE_DIRECTORY "${AZMUITH_SHADER_OUTPUT_DIR}")
    foreach(SHADER viewport.vert viewport.frag post.vert post.frag shadow.vert)
        add_custom_command(
            OUTPUT "${AZMUITH_SHADER_OUTPUT_DIR}/${SHADER}.spv"
            COMMAND "${GLSLC_EXECUTABLE}" --target-env=vulkan1.1
                "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/${SHADER}"
                -o "${AZMUITH_SHADER_OUTPUT_DIR}/${SHADER}.spv"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/${SHADER}"
            VERBATIM)
        list(APPEND AZMUITH_SHADER_OUTPUTS "${AZMUITH_SHADER_OUTPUT_DIR}/${SHADER}.spv")
    endforeach()
    add_custom_target(azmuith_viewport_shaders DEPENDS ${AZMUITH_SHADER_OUTPUTS})
    add_dependencies(azmuith_sandbox azmuith_viewport_shaders)
    target_compile_definitions(azmuith_sandbox PRIVATE
        AZMUITH_GPU_VIEWPORT=1
        AZMUITH_SHADER_DIR=\"${AZMUITH_SHADER_OUTPUT_DIR}\")
else()
    target_compile_definitions(azmuith_sandbox PRIVATE AZMUITH_GPU_VIEWPORT=0)
endif()
if(AZMUITH_ENABLE_PHYSX_GPU)
    target_compile_definitions(azmuith_sandbox PRIVATE AZMUITH_ENABLE_PHYSX_GPU=1 PX_SUPPORT_GPU_PHYSX=1)
    target_link_libraries(azmuith_sandbox PRIVATE PhysXGpu)
endif()

if(MSVC)
    target_compile_options(azmuith_sandbox PRIVATE /W4 /permissive-)
else()
    target_compile_options(azmuith_sandbox PRIVATE -Wall -Wextra -Wpedantic)
endif()