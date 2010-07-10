CC = g++
LIB = "../Library/libBox2D.a"
FLAGS = -lglut

all: simulator 

simulator: simulator.cpp
	$(CC) -o $@ $^ $(FLAGS) $(LIB)
