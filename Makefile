CC = colorgcc #sudo apt-get install colorgcc
#CC = gcc
FLAGS = -Wall -I$(ROBOPET_PATH) -I$(COMMUNICATION_H) -I$(SOCKETS_PATH) -lstdc++ -lprotobuf

ROBOPET_PATH = ../lib
ROBOPET_LIB = $(ROBOPET_PATH)/robopet.a #$(AI_PATH)/point.o $(AI_PATH)/vector.o $(AI_PATH)/movingObject.o $(AI_PATH)/physicsRobot.o

COMMUNICATION_PATH = ../robopet-communication/
COMMUNICATION_H = $(COMMUNICATION_PATH)/packets
COMMUNICATION_LIB = $(COMMUNICATION_PATH)/communication.a

SOCKETS_PATH = $(COMMUNICATION_PATH)/socket

all: simulator

simulator: main.cpp $(ROBOPET_LIB) $(COMMUNICATION_LIB)
	@echo $@
	@$(CC) -o $@ $^ $(FLAGS) `pkg-config --cflags --libs protobuf`