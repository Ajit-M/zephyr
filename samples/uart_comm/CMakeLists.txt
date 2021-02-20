# SPDX-License-Identifier: Apache-2.0

# cmake_minimum_required(VERSION 3.13.1)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(uart_comm)




# target_sources(app PRIVATE src/main.c        
# )
# target_sources(app PRIVATE src/frsky.c)
# target_sources(app PRIVATE src/frsky.h)

# add_library(
#     frsky STATIC
#     src/frsky.c
#     src/frsky.h
# )

# add_executable(uart_comm main.c)

# target_link_libraries(app PRIVATE frsky)



################################################

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_comm)

target_sources(app PRIVATE 
src/main.c
# src/uart_comm.c
)

