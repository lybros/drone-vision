cmake_minimum_required(VERSION 2.8)

add_definitions(-DSQLITE_THREADSAFE=1)

add_library(sqlite sqlite3.h sqlite3ext.h sqlite3.c)
target_link_libraries(sqlite pthread dl)
