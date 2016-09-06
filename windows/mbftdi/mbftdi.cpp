// mbftdi.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"

//============================================================================
//  Use of FTDI D2XX library:
//----------------------------------------------------------------------------
//  Include the following 2 lines in your header-file
#pragma comment(lib, "FTD2XX.lib")
#include "FTD2XX.h"
//============================================================================

extern "C" int play_svf(FILE* f);
extern "C" int ver_major;
extern "C" int ver_minor;

int _tmain(int argc, _TCHAR* argv[])
{
	printf("mbftdi v%d.%d - burn MAX2 CPLD from Altera Vector Programming File *.svf\n",ver_major,ver_minor);
	printf("FTDI port to JTAG is used for programming\n");
	printf("Usage example: mbftdi myfile.svf\n\n");

	if(argc<2)
	{
		printf("No input parameters!\n");
		return -1;
	}

	//open file named in input parameter
	FILE* f = _wfopen(argv[1],_T("r"));
	if(f==NULL)
	{
		printf("Cannot open SVF file %s\n",argv[2]);
		return -1;
	}

	play_svf(f);

	fclose(f);
	return 0;
}
