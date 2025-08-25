#!/bin/bash
cp test/GRID.txt .
g++ -o testMain \
-D NO_DEBUG \
-I test \
-I src/ \
-I lib/Util \
-I lib/MathStuff/ \
-I lib/Algo/ \
-I lib/Stuff/ \
test/testMain.cpp \
test/vector2.cpp \
test/vector2i.cpp \
src/AbstractMST.cpp \
src/FlowField.cpp \
src/GDDistanceMap.cpp \
src/GridToGraph.cpp \
src/GridTypes.cpp \
src/Router.cpp \
src/Routing.cpp \
src/WallDistanceGrid.cpp \
lib/Stuff/TGA.cpp \
lib/Algo/ZSThinning.cpp \
-o testMain.exe
