CC=gcc

#LIBPATH = ./raspberry
#INCPATH = ./raspberry
LIBPATH = ./x64
INCPATH = ./x64

CFLAGS  = $(LIBPATH)/libftd2xx.a
CFLAGS += -ldl -lrt -lpthread
CFLAGS += -I$(INCPATH)

APP = mbftdi
all: $(APP)

$(APP): mbftdi.c ../common/mblastercore.c
	$(CC) mbftdi.c ../common/mblastercore.c -o $(APP) $(CFLAGS) 

clean:
	rm -f *.o ; rm $(APP)
