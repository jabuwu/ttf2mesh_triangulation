cmake_minimum_required(VERSION 3.0)
project(ttf2mesh_data_test_gen C)

add_executable(ttf2mesh_test_data_gen ttf2mesh_test_data_gen.c ttf2mesh/ttf2mesh.c)
target_include_directories(ttf2mesh_test_data_gen PRIVATE ttf2mesh)
target_compile_definitions(ttf2mesh_test_data_gen PRIVATE
    -DFONT_DIR="${CMAKE_SOURCE_DIR}/fonts/"
    -DTEST_DIR="${CMAKE_SOURCE_DIR}/../src/test/"
)
