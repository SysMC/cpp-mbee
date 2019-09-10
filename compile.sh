#!/bin/bash
# Command file for compiling application based on MBee C++ library.
# First command line argument must be full name of C++ source file.
# Second command line argument is the name of output file.
g++ MBee.cpp System.cpp LinuxSerial.cpp SerialStar.cpp $1 -o $2