#!/bin/bash
g++ -o testMain \
-I . \
-I ../src/ \
-I ../lib/Util \
-I ../lib/MathStuff/ \
-I ../lib/Algo/ \
-I ../lib/Stuff/ \
testMain.cpp \
vector2.cpp \
vector2i.cpp \
../src/AbstractMST.cpp \
../src/FlowField.cpp \
../src/GDDistanceMap.cpp \
../src/GridToGraph.cpp \
../src/GridTypes.cpp \
../src/Router.cpp \
../src/Routing.cpp \
../src/WallDistanceGrid.cpp \
../lib/Stuff/TGA.cpp \
../lib/Algo/ZSThinning.cpp 
