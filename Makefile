CC = colorgcc #sudo apt-get install colorgcc
#CC = g++

COMMUNICATION_PATH = ../robopet-communication/
COMMUNICATION_H = $(COMMUNICATION_PATH)/packets
COMMUNICATION_LIB = $(COMMUNICATION_PATH)/communication.a

SOCKETS_PATH = $(COMMUNICATION_PATH)/socket

INCLUDE_PATH = /usr/local/include/Box2D

ROBOPET_PATH = ../lib
ROBOPET_LIB = $(ROBOPET_PATH)/robopet.a
BOX2D_LIB = /usr/local/lib/libBox2D.a

FLAGS = -Wall -I$(ROBOPET_PATH) -I$(COMMUNICATION_H) -I$(SOCKETS_PATH) -I$(INCLUDE_PATH)  -lprotobuf -lglut -lstdc++


all: simulator

simulator: simulation.o main.cpp
	@echo $@
	@$(CC) -o $@ $^ $(FLAGS) $(ROBOPET_LIB) $(BOX2D_LIB) $(COMMUNICATION_LIB) `pkg-config --cflags --libs protobuf`

simulation.o: simulation.cpp
	@echo $@
	@$(CC) -c -o $@ $^ $(FLAGS)
