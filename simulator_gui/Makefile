CC = g++

LIB_PATH = ./Include/

FLAGS = -I$(LIB_PATH) -lglut -lbox2d

all: simulator 

simulator: simulator.cpp
	$(CC) -o $@ $^ $(FLAGS)
