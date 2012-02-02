#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>

#include "../common/ftd2xx.h"

#define false 0
#define true 1

extern int play_svf(FILE* f);
extern int ver_major;
extern int ver_minor;

int main(int argc, char* argv[])
{
	printf("mbftdi v%d.%d - execute Altera Vector Programming File *.svf\n",ver_major,ver_minor);
	printf("FTDI port to JTAG is used for programming\n");
	printf("Usage example: >mbftdi myfile.svf\n\n");

	if (argc < 2)
	{
		fprintf(stderr, "No input parameters!\n");
		exit(EXIT_FAILURE);
	}

	//open file
	FILE* f = fopen(argv[1], "r");

	if (f == NULL)
	{
		fprintf(stderr, "Cannot open SVF file %s: %s\n", argv[1], strerror(errno));
		exit(EXIT_FAILURE);
	}

	play_svf(f);

	int ret = fclose(f);
	if (ret != 0)
	{
		fprintf(stderr, "The SVF file was not closed normally: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
	exit(EXIT_SUCCESS);
}
