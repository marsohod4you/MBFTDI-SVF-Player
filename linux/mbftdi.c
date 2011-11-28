
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../common/ftd2xx.h"

#define false 0
#define true 1

extern int play_svf(FILE* f);
extern int ver_major;
extern int ver_minor;

int main(int argc, char* argv[])
{
	printf("mbftdi v%d.%d - burn MAX2 CPLD from Altera Vector Programming File *.svf\n",ver_major,ver_minor);
	printf("FTDI port to JTAG is used for programming\n");
	printf("Usage example: >mbftdi myfile.svf\n\n");

	if(argc<2)
	{
		printf("No input parameters!\n");
		return -1;
	}

	//open file
	FILE* f = fopen( argv[1],"r");

	if(f==NULL)
	{
		printf("Cannot open SVF file %s\n",argv[2]);
		return -1;
	}

	play_svf(f);

	fclose(f);
	return 0;
}
