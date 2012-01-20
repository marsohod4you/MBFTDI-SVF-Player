CC=gcc

LIBPATH = /home/nick/myftdi/x86_64
INCPATH = /home/nick/myftdi/


CFLAGS  =-ldl -lrt -lpthread
CFLAGS += $(LIBPATH)/libftd2xx.a
CFLAGS += -I../common
CFLAGS += -I$(INCPATH)

APP = mbftdi
all: $(APP)

$(APP): mbftdi.c ../common/mblastercore.c
	$(CC) mbftdi.c ../common/mblastercore.c -o $(APP) $(CFLAGS) 

clean:
	rm -f *.o ; rm $(APP)
