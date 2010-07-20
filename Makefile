CC = colorgcc #sudo apt-get install colorgcc
#CC = g++
FLAGS = -lglut

COMMUNICATION_PATH = ../robopet-communication/
COMMUNICATION_H = $(COMMUNICATION_PATH)/packets
COMMUNICATION_LIB = $(COMMUNICATION_PATH)/communication.a

SOCKETS_PATH = $(COMMUNICATION_PATH)/socket

LIB_PATH = ./Include/


FLAGS = -Wall -I$(ROBOPET_PATH) -I$(COMMUNICATION_H) -I$(SOCKETS_PATH) -I$(LIB_PATH)  -lprotobuf -lbox2d

ROBOPET_PATH = ../lib
ROBOPET_LIB = $(ROBOPET_PATH)/robopet.a


all: simulator

simulator: main.cpp $(ROBOPET_LIB) $(COMMUNICATION_LIB)
	@echo $@
	@$(CC) -o $@ $^ $(FLAGS) `pkg-config --cflags --libs protobuf`
