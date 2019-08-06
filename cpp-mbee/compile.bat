@echo off
rem Command file for compiling application based on MBee C++ library.
rem First command line argument must be full name of C++ source file.
rem Second command line argument is the name of output exe file without extension.

g++ MBee.cpp System.cpp WindowsSerial.cpp SerialStar.cpp %1 -o %2.exe

rem usage example: compile.bat c:\cpp-mbee\examples\WindowsLinux\IoSamples_Callbacks\IoSamples_Callbacks.cpp IoSamples_Callbacks