CC = colorgcc #sudo apt-get install colorgcc
#CC = g++
FLAGS = -lglut

COMMUNICATION_PATH = ../robopet-communication/
COMMUNICATION_H = $(COMMUNICATION_PATH)/packets
COMMUNICATION_LIB = $(COMMUNICATION_PATH)/communication.a

SOCKETS_PATH = $(COMMUNICATION_PATH)/socket

INCLUDE_PATH = /usr/local/include/Box2D
LIB_PATH = /usr/local/lib/libBox2D.a

FLAGS = -Wall -I$(ROBOPET_PATH) -I$(COMMUNICATION_H) -I$(SOCKETS_PATH) -I$(INCLUDE_PATH)  -lprotobuf $(LIB_PATH)

ROBOPET_PATH = ../lib
ROBOPET_LIB = $(ROBOPET_PATH)/robopet.a


all: simulator

simulator: main.cpp $(ROBOPET_LIB) $(COMMUNICATION_LIB)
	@echo $@
	@$(CC) -o $@ $^ $(FLAGS) `pkg-config --cflags --libs protobuf`
