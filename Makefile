# single_chan_pkt_fwd
# Single Channel LoRaWAN Gateway

CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: loragateway

loragateway: main.o
	$(CC) main.o $(LIBS) -o loragateway

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

clean:
	rm *.o loragateway	
