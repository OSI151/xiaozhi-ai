# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

# 版本号规则：V224 = 基于官方 xiaozhi-esp32 2.2.4，横杠后是本项目的能力阶段与序号。
#   1.x 能亮能动 / 2.x 能交互 / 3.x 有感知 / 4.x 有性格 / 5.x 好用可发布
#   每加一个落到对应阶段的功能就 +1，详见 README 的版本表。
set(PROJECT_VER "V224-5.2")
project(xiaozhi)
