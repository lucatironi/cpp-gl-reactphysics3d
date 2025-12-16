include(FetchContent)

# glfw
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(glfw)

# glad
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v2.0.8
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR cmake)

FetchContent_MakeAvailable(glad)

glad_add_library(glad STATIC REPRODUCIBLE LOADER API gl:core=4.1)

# glm
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.2
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(glm)

# stb
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    CONFIGURE_COMMAND ""
    BUILD_COMMAND "")

FetchContent_MakeAvailable(stb)

add_library(stb INTERFACE ${stb_SOURCE_DIR}/stb_image.h)
target_include_directories(stb INTERFACE ${stb_SOURCE_DIR})

# ReactPhysics3D
FetchContent_Declare(
    ReactPhysics3D
    GIT_REPOSITORY https://github.com/DanielChappuis/reactphysics3d.git
    GIT_TAG v0.10.2
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(ReactPhysics3D)