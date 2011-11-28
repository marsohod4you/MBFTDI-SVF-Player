
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WINDOWS
#include <windows.h>
#endif

#include "ftd2xx.h"

#define true 1
#define false 0

//version variables
int ver_major = 1;
int ver_minor = 0;

#ifndef _WINDOWS

void Sleep(int ms)
{
usleep(ms*1000);
}

#endif

//////////////////////////////////////////////////////////////////////////////
// native FTDI stuff
//////////////////////////////////////////////////////////////////////////////

DWORD dwNumDevs; // The number of devices
unsigned int uiDevIndex = 0xF; // The device in the list that is used
DWORD dwCount = 0; // General loop index
FT_HANDLE ftHandle; // Handle of the FTDI device
FT_STATUS ftStatus; // Result of each D2XX call
BYTE byOutputBuffer[1024]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
BYTE byInputBuffer[1024]; // Buffer to hold data read from the FT2232H
DWORD dwNumBytesToSend = 0; // Index to the output buffer
DWORD dwNumBytesSent = 0; // Count of actual bytes sent - used with FT_Write
DWORD dwNumBytesToRead = 0; // Number of bytes available to read in the driver's input buffer
DWORD dwNumBytesRead = 0; // Count of actual bytes read - used with FT_Read
DWORD dwClockDivisor = 29; // Value of clock divisor, SCL Frequency = 60/((1+vl)*2) (MHz)

int ftdi_init()
{
// Does an FTDI device exist?
printf("Checking for FTDI devices...\n");
ftStatus = FT_CreateDeviceInfoList(&dwNumDevs);

// Get the number of FTDI devices
if (ftStatus != FT_OK) // Did the command execute OK?
{
	printf("Error in getting the number of devices\n");
	return 1; // Exit with error
}

if (dwNumDevs < 1) // Exit if we don't see any
{
	printf("There are no FTDI devices installed\n");
	return 1; // Exist with error
}

printf("%d FTDI devices found - the count includes individual ports on a single chip\n", dwNumDevs);

// Open the port - For this application note, assume the first device is a FT2232H or FT4232H
// Further checks can be made against the device descriptions, locations, serial numbers, etc.
// before opening the port.
printf("Assume first device has the MPSSE and open it...\n");
ftStatus = FT_Open(0, &ftHandle);
if (ftStatus != FT_OK)
{
	printf("Open Failed with error %d\n", ftStatus);
	printf("If runing on Linux then try <rmmod ftdi_sio> first\n");
	return 1; // Exit with error
}

// Configure port parameters
printf("Configuring port for MPSSE use...\n");

//Reset USB device
ftStatus |= FT_ResetDevice(ftHandle);

//Purge USB receive buffer first by reading out all old data from FT2232H receive buffer

// Get the number of bytes in the FT2232H receive buffer
ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);

//Read out the data from FT2232H receive buffer
if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))
	FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);

//Set USB request transfer sizes to 64K
ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);

//Disable event and error characters
ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);

//Sets the read and write timeouts in milliseconds
ftStatus |= FT_SetTimeouts(ftHandle, 0, 5000);

//Set the latency timer (default is 16mS)
ftStatus |= FT_SetLatencyTimer(ftHandle, 16);

ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00);

//Reset controller
ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);

//Enable MPSSE mode
if (ftStatus != FT_OK)
{
	printf("Error in initializing the MPSSE %d\n", ftStatus);
	FT_Close(ftHandle);
	return 1; // Exit with error
}

//return with success
return 0;
}

int configure_mpsse()
{
BOOL bCommandEchod;
// -----------------------------------------------------------
// Synchronize the MPSSE by sending a bogus opcode (0xAA),
// The MPSSE will respond with "Bad Command" (0xFA) followed by
// the bogus opcode itself.
// -----------------------------------------------------------
// Reset output buffer pointer
dwNumBytesToSend=0;
//Add bogus command ‘xAA’ to the queue
byOutputBuffer[dwNumBytesToSend++] = 0xAA;
// Send off the BAD commands
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
do
{
	// Get the number of bytes in the device input buffer
	ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));

//Read out the data from input buffer
ftStatus = FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);

//Check if Bad command and echo command received
bCommandEchod = false;
for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)
{
	if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount+1] == 0xAA))
	{
		bCommandEchod = true;
		break;
	}
}

if (bCommandEchod == false)
{
	printf("Error in synchronizing the MPSSE\n");
	FT_Close(ftHandle);
	return 1; // Exit with error
}

// -----------------------------------------------------------
// Configure the MPSSE settings for JTAG
// Multiple commands can be sent to the MPSSE with one FT_Write
// -----------------------------------------------------------

// Set up the Hi-Speed specific commands for the FTx232H

// Start with a fresh index
dwNumBytesToSend = 0;

// Use 60MHz master clock (disable divide by 5)
byOutputBuffer[dwNumBytesToSend++] = 0x8A;
// Turn off adaptive clocking (may be needed for ARM)
byOutputBuffer[dwNumBytesToSend++] = 0x97;
// Disable three-phase clocking
byOutputBuffer[dwNumBytesToSend++] = 0x8D;
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

// Send off the HS-specific commands

dwNumBytesToSend = 0;
// Set initial states of the MPSSE interface - low byte, both pin directions and output values
// Pin name Signal Direction Config Initial State Config
// ADBUS0 TCK output 1 low 0
// ADBUS1 TDI output 1 low 0
// ADBUS2 TDO input 0 0
// ADBUS3 TMS output 1 high 1
// ADBUS4 GPIOL0 input 0 0
// ADBUS5 GPIOL1 input 0 0
// ADBUS6 GPIOL2 input 0 0
// ADBUS7 GPIOL3 input 0 0
// Set data bits low-byte of MPSSE port
byOutputBuffer[dwNumBytesToSend++] = 0x80;
// Initial state config above
byOutputBuffer[dwNumBytesToSend++] = 0x08;
// Direction config above
byOutputBuffer[dwNumBytesToSend++] = 0x0B;
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

// Send off the low GPIO config commands
dwNumBytesToSend = 0;
// Set initial states of the MPSSE interface - high byte, both pin directions and output values
// Pin name Signal Direction Config Initial State Config
// ACBUS0 GPIOH0 input 0 0
// ACBUS1 GPIOH1 input 0 0
// ACBUS2 GPIOH2 input 0 0
// ACBUS3 GPIOH3 input 0 0
// ACBUS4 GPIOH4 input 0 0
// ACBUS5 GPIOH5 input 0 0
// ACBUS6 GPIOH6 input 0 0
// ACBUS7 GPIOH7 input 0 0
// Set data bits low-byte of MPSSE port
byOutputBuffer[dwNumBytesToSend++] = 0x82;
// Initial state config above
byOutputBuffer[dwNumBytesToSend++] = 0x00;
// Direction config above
byOutputBuffer[dwNumBytesToSend++] = 0x00;
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

// Send off the high GPIO config commands
dwNumBytesToSend = 0;
// Set TCK frequency
// TCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2)
//Command to set clock divisor
byOutputBuffer[dwNumBytesToSend++] = 0x86;
//Set 0xValueL of clock divisor
byOutputBuffer[dwNumBytesToSend++] = dwClockDivisor & 0xFF;
//Set 0xValueH of clock divisor
byOutputBuffer[dwNumBytesToSend++] = (dwClockDivisor >> 8) & 0xFF;
// Send off the clock divisor commands
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

dwNumBytesToSend = 0;
// Disable internal loop-back
// Disable loopback
byOutputBuffer[dwNumBytesToSend++] = 0x85;
// Send off the loopback command
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
return ftStatus;
}

// -----------------------------------------------------------
// Start closing everything down
// -----------------------------------------------------------
int close_ftdi()
{
printf("\nJTAG program executed successfully.\n");
printf("Press <Enter> to continue\n");
FT_Close(ftHandle); // Close the port
return 0;
}

//////////////////////////////////////////////////////////////////////////////
// buffered write to FTDI chip
// then larger send blocks then faster can be solution 
//////////////////////////////////////////////////////////////////////////////

//define buffer accumulating commands to be sent
#define WR_BUF_SIZE 512
unsigned char wr_buffer[WR_BUF_SIZE*2];
int wr_buf_ptr = 0;

//write to ftdi all accumulated commands
int flush()
{
	DWORD wr_ptr = 0;
	while( wr_ptr != wr_buf_ptr )
	{
		DWORD written = 0;
		ftStatus = FT_Write(ftHandle,&wr_buffer[wr_ptr],wr_buf_ptr-wr_ptr,&written);
		if(ftStatus)
			return -1; //error
		wr_ptr+=written;
	}

	wr_buf_ptr = 0;
	//FT_Purge(ftHandle,FT_PURGE_TX);
	return 0;
}

//fake write function which stores sending data into accummulating buffer instead of writing to ftdi
FT_STATUS FT_Write_b(
    FT_HANDLE ftHandle,
    LPVOID lpBuffer,
    DWORD dwBytesToWrite,
    LPDWORD lpBytesWritten
    )
{
	memcpy(&wr_buffer[wr_buf_ptr],lpBuffer,dwBytesToWrite);
	wr_buf_ptr += dwBytesToWrite;
	if(wr_buf_ptr>WR_BUF_SIZE)
		flush(); //we have accumulated enough to flush
	*lpBytesWritten = dwBytesToWrite;
	return 0;
}

// Navigage TMS to desired state with no TDO read back
// please do 0<num_shifts<8
void tms_command_nr(unsigned char num_shifts, unsigned char pattern, unsigned char least_bit)
{
dwNumBytesToSend = 0;
// Don't read data
byOutputBuffer[dwNumBytesToSend++] = 0x4B;
// Number of clock pulses, zero value mean 1 shift
byOutputBuffer[dwNumBytesToSend++] = num_shifts-1;
// Data is shifted LSB first, 8th bit of value will be used as least tdi bit
byOutputBuffer[dwNumBytesToSend++] = pattern | (least_bit<<7);
// Send off the TMS command
ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
}


//clear input buffer by reading it
void read_all()
{
    int i,num;
    for(i=0; i<16; i++)
    {
	ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	if(ftStatus == FT_OK && dwNumBytesToRead>0)
	{
	    num=dwNumBytesToRead;
	    if(num>sizeof(byInputBuffer))
		num=sizeof(byInputBuffer);
	    ftStatus = FT_Read(ftHandle, byInputBuffer, num, &dwNumBytesRead);
	}
	else
	    break;
    }
}

//buffers where we store tdo expected answer to be compared with tdo real answer
#define EXP_BUF_LEN 1024
#define EXP_BUF_THRESH (EXP_BUF_LEN/8)
unsigned char expect_buffer[EXP_BUF_LEN+16];
unsigned char expect_buffer_mask[EXP_BUF_LEN+16];
unsigned char compare_buffer[EXP_BUF_LEN+16];
int exp_buff_ptr = 0;

//we have "expect buffer"
//and we read data from ftdi chip
//result should be equal
//but we do this check sometimes only - sending bigger packets should increase performance
int check_answer(BOOL force)
{
	int i;
	unsigned char exp_val,got_val;

	//return if nothing to check
	if(exp_buff_ptr==0)
		return 1;

	if( (exp_buff_ptr<EXP_BUF_THRESH) && (!force) )
		return 1; //can skip check now, do it later

	//finally write all pending commands to ftdi chip
	flush();

	//wait until we have in read buffer all reply data
	for(i=0; i<64; i++)
	{
		do
		{
			ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
			// Get the number of bytes in the device input buffer
		} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK)); //or Timeout
		if(dwNumBytesToRead==exp_buff_ptr)
			break;
		Sleep(10);
	}

	if(dwNumBytesToRead!=exp_buff_ptr)
	{
		printf("oops.. expect %d bytes read but got %d\n",exp_buff_ptr,dwNumBytesToRead);
	}

	//Read out the data from input buffer
	memset(compare_buffer,0,dwNumBytesToRead);
	ftStatus = FT_Read(ftHandle, compare_buffer, dwNumBytesToRead, &dwNumBytesRead);

	//check what we had read and what is expected
	//on every 16bit word we should receive 3 byte answer (3bytes convert to 16bit word)
	//so compare 16bit word with another expected 16bit word
	for(i=0; i<exp_buff_ptr; i++)
	{
		got_val = compare_buffer[i] & expect_buffer_mask[i];
		exp_val = expect_buffer[i]  & expect_buffer_mask[i];
		if( exp_val != got_val )
		{
			exp_buff_ptr=0;
			return 0;
		}
	}

//answer was checked, so we can accumulate expected data again
exp_buff_ptr=0;
return 1;
}

//////////////////////////////////////////////////////////////////////////////
// SVF functions
//////////////////////////////////////////////////////////////////////////////

//go to Test Logic Reset state then to Idle
void goto_tlr()
{
	// Navigage TMS to Test-Logic-Reset (TMS=1 more then 4 times) then Idle (TMS=0 once)
	// // Data is shifted LSB first, so: 011111 >>> chip
	tms_command_nr(6,0x1f,0);	
}

//current state is Idle and stay Idle for some clocks
void Idle()
{
	// Idle (TMS=0 any times)
	// Data is shifted LSB first, so the TMS pattern is 000000 >>> chip
	tms_command_nr(6,0,0);
}

int sir(int nclk, int val)
{
	if(check_answer(true)==0)
	{
		printf("error on check answer\n");
		return 0;
	}

	if(nclk<10)
	{
		printf("error SIR parameters\n");
		return 0;
	}

	//enter to state where we shifh IR
	tms_command_nr(4,0x03,0);

	dwNumBytesToSend = 0;
	byOutputBuffer[dwNumBytesToSend++] = 0x1b;
	byOutputBuffer[dwNumBytesToSend++] = 7;
	byOutputBuffer[dwNumBytesToSend++] = val&0xFF;
	byOutputBuffer[dwNumBytesToSend++] = 0x1b;
	byOutputBuffer[dwNumBytesToSend++] = (nclk-8)-2;
	byOutputBuffer[dwNumBytesToSend++] = (val>>8)&0xFF;
	ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

	//go back from IR shift to Idle
	tms_command_nr(3,0x03,((val>>(nclk-1))&1));
	return 0;
}

int runidle(int num)
{
	int num_ = num;
	if(num>8)
	{
		while(1)
		{
			if( num >= (0xFFFF+1)*8 )
			{
				dwNumBytesToSend = 0;
				byOutputBuffer[dwNumBytesToSend++] = 0x8F;
				byOutputBuffer[dwNumBytesToSend++] = 0xFF;
				byOutputBuffer[dwNumBytesToSend++] = 0xFF;
				ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
				num = num - (0xFFFF+1)*8;
				continue;
			}
			else
			{
				if(num>8)
				{
					int n = num/8-1;
					num = num - (n+1)*8;
					dwNumBytesToSend = 0;
					byOutputBuffer[dwNumBytesToSend++] = 0x8F;
					byOutputBuffer[dwNumBytesToSend++] = n&0xFF;
					byOutputBuffer[dwNumBytesToSend++] = (n>>8)&0xFF;
					ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
				}
				break;
			}
		}
	}

	if(num)
	{
		dwNumBytesToSend = 0;
		byOutputBuffer[dwNumBytesToSend++] = 0x8e;
		byOutputBuffer[dwNumBytesToSend++] = num-1;
		// Send off
		ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	}

	if(num_>1000000)
	{
		flush();
		Sleep(500);
	}
	return ftStatus;
}

#define MAX_STRING_LENGTH (1024*4)
char rbuffer[MAX_STRING_LENGTH];
unsigned char rbytes[MAX_STRING_LENGTH];
char command[MAX_STRING_LENGTH];
int line;

//process typical strings:
//SIR 10 TDI (203);
void do_SIR()
{
	//get command parameters
	unsigned int sir_arg = 0;
	unsigned int tdi_arg = 0;
	int n = sscanf(rbuffer,"SIR %d TDI (%X);",&sir_arg,&tdi_arg);
	if(n!=2)
	{
		printf("error processing SIR\n");
	}
	sir(sir_arg,tdi_arg);
}

//////////////////////////////////////////////////////////////////////////////
// memory alloc/free functions
//////////////////////////////////////////////////////////////////////////////

//global pointers to sdr tdi, tdo, mask and smask arrays
//we expect that dr data can be very long, but we do not know exact size
//size of arrays allocated will be defined during SVF strings parcing
unsigned char* psdr_tdi_data   = NULL;
unsigned char* psdr_tdo_data   = NULL;
unsigned char* psdr_mask_data  = NULL;
unsigned char* psdr_smask_data = NULL;
unsigned int sdr_data_size = 0; //current size of arrays
unsigned int sdr_tdi_sz;
unsigned int sdr_tdo_sz;
unsigned int sdr_mask_sz;
unsigned int sdr_smask_sz;

//allocate arrays for sdr tdi, tdo and mask
//return 1 if ok or 0 if fail
unsigned int alloc_sdr_data(unsigned int size)
{
	//compare new size with size of already allocated buffers
	if(sdr_data_size>=size)
		return 1; //ok, because already allocated enough

	//we need to allocate memory for arrays
	//but first free previously allocated buffers

	//tdi
	if(psdr_tdi_data)
		{ free(psdr_tdi_data); psdr_tdi_data=NULL; }

	//tdo
	if(psdr_tdo_data)
		{ free(psdr_tdo_data); psdr_tdo_data=NULL; }

	//mask
	if(psdr_mask_data)
		{ free(psdr_mask_data); psdr_mask_data=NULL; }

	//smask
	if(psdr_smask_data)
		{ free(psdr_smask_data); psdr_smask_data=NULL; }

	psdr_tdi_data = (unsigned char*)malloc(size);
	if(psdr_tdi_data==NULL)
	{
		printf("error allocating sdr tdi buffer\n");
		return(0);
	}

	psdr_tdo_data = (unsigned char*)malloc(size);
	if(psdr_tdo_data==NULL)
	{
		free(psdr_tdi_data);
		psdr_tdi_data=NULL;
		printf("error allocating sdr tdo buffer\n");
		return(0);
	}

	psdr_mask_data = (unsigned char*)malloc(size);
	if(psdr_mask_data==NULL)
	{
		free(psdr_tdi_data);
		free(psdr_tdo_data);
		psdr_tdi_data=NULL;
		psdr_tdo_data=NULL;
		printf("error allocating sdr mask buffer\n");
		return(0);
	}

	psdr_smask_data = (unsigned char*)malloc(size);
	if(psdr_smask_data==NULL)
	{
		free(psdr_tdi_data);
		free(psdr_tdo_data);
		free(psdr_mask_data);
		psdr_tdi_data=NULL;
		psdr_tdo_data=NULL;
		psdr_mask_data=NULL;
		printf("error allocating sdr smask buffer\n");
		return(0);
	}

	//remember that we have allocated some size memory
	sdr_data_size = size;

	//we have successfully allocated buffers for sdr data!
	return 1;
}

//free buffers allocated for sdr tdi, tdo, mask
void free_sdr_data()
{
	if(psdr_tdi_data)
		free(psdr_tdi_data);
	if(psdr_tdo_data)
		free(psdr_tdo_data);
	if(psdr_mask_data)
		free(psdr_mask_data);
	if(psdr_smask_data)
		free(psdr_smask_data);
	sdr_data_size = 0;
}

#define BLOCK_SZ 128

//send some data to DR
//please num_bytes non zero
//offset points into sdr tdi array and func returns updated offset
unsigned int send_sdr_block(unsigned int num_bytes, unsigned int offset, unsigned int has_tdo, unsigned int has_mask)
{
	unsigned char block_tdi[BLOCK_SZ+1];
	unsigned char cmd_mask;
	unsigned char b;
	unsigned int  i,j;

	//prepare command modifier which says do we expect answer or not
	if(has_tdo)
		cmd_mask = 0x20;
	else
		cmd_mask = 0x00;

	dwNumBytesToSend = 0;
	byOutputBuffer[dwNumBytesToSend++] = 0x19 | cmd_mask;
	byOutputBuffer[dwNumBytesToSend++] =  (num_bytes-1) & 0xFF;
	byOutputBuffer[dwNumBytesToSend++] = ((num_bytes-1) >>8 ) & 0xFF;
	ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
		
	//write data, but firstreverse bytes of array
	j = offset;		
	for(i=0; i<num_bytes; i++)
	{
		//gather byte from two thetrades of sdr array
		b = psdr_tdi_data[j--];
		b = b | (psdr_tdi_data[j--]<<4);
		block_tdi[i] = b;
	}
	ftStatus = FT_Write_b(ftHandle, block_tdi, num_bytes, &dwNumBytesSent);

	//should we expect and compare answer from chip?
	if(has_tdo)
	{
		//we should read from chip TDO answer and compare with expected
		//prepare array for comparation

		j = offset;		
		for(i=exp_buff_ptr; i<exp_buff_ptr+num_bytes; i++)
		{
			//save tdo byte
			b = psdr_tdo_data[j];
			b = b | (psdr_tdo_data[j-1]<<4);
			expect_buffer[i] = b;

			//probably we have also tdo mask
			if(has_mask)
			{
				b = psdr_mask_data[j];
				b = b | (psdr_mask_data[j-1]<<4);
				expect_buffer_mask[i] = b;
			}
			else
				expect_buffer_mask[i] = 0xff;

			j -= 2;
		}
		exp_buff_ptr+=num_bytes;
		check_answer(false);
	}

	//offset is index in tdi array which consists of thetrades, that is why mul 2
	offset -= num_bytes*2;
	return offset;
}

int sdr_nbits(unsigned int nbits, unsigned int has_tdo, unsigned int has_mask, unsigned int has_smask)
{
	unsigned char val_o, val_i, val_m, m;
	unsigned char cmd_mask;
	unsigned int  num_bytes;
	unsigned int  offset,i,j;

	if(nbits<1)
		return 0;

	//go to state where we can shift DR
	tms_command_nr(3,0x01,0);

	//calc "offset" - index of least thetrade from array of tdi data
	offset = sdr_tdi_sz-1;

	//first send large blocks if we have
	while( (nbits-1) > BLOCK_SZ*8 )
	{
		offset = send_sdr_block(BLOCK_SZ,offset,has_tdo,has_mask);
		nbits -= BLOCK_SZ*8;
	}

	//send smaller block but leave at least one byte
	if(nbits>8)
	{
		num_bytes = (nbits-1) / 8;
		offset = send_sdr_block(num_bytes,offset,has_tdo,has_mask);
		nbits -= num_bytes*8;
	}

	//get last byte from tdi, tdo, mask arrays
	val_i  = psdr_tdi_data[offset];
	val_i |= psdr_tdi_data[offset-1]<<4;

	val_o  = psdr_tdo_data[offset];
	val_o |= psdr_tdo_data[offset-1]<<4;

	val_m  = psdr_mask_data[offset];
	val_m |= psdr_mask_data[offset-1]<<4;

	//define we need read answer or not
	cmd_mask = 0;
	if(has_tdo) cmd_mask = 0x20;

	if(nbits>1)
	{
		nbits--;

		//send some least bits but not last
		dwNumBytesToSend = 0;
		byOutputBuffer[dwNumBytesToSend++] = 0x1b | cmd_mask;
		byOutputBuffer[dwNumBytesToSend++] = nbits-1;
		byOutputBuffer[dwNumBytesToSend++] = val_i;
		ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

		if(has_tdo)
		{
			//because we had sent nbits we will get answer of same nbits coming LSB first
			expect_buffer[exp_buff_ptr] = (val_o<<(8-nbits));
			expect_buffer_mask[exp_buff_ptr] = (val_m<<(8-nbits));
			exp_buff_ptr++;
		}
	}

	//send last bit with TMS command
	dwNumBytesToSend = 0;
	byOutputBuffer[dwNumBytesToSend++] = 0x4B | cmd_mask;
	// Number of clock pulses = Length + 1 (1 clocks here)
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	// Data is shifted LSB first, so the TMS pattern is 110
	byOutputBuffer[dwNumBytesToSend++] = 0x01 | ( ((val_i>>nbits)&1)<<7 );
	// Send off the TMS command
	ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);

	if(has_tdo)
	{
		expect_buffer[exp_buff_ptr] = (val_o >> nbits)<<7;
		expect_buffer_mask[exp_buff_ptr] = (val_m >> nbits)<<7;
		exp_buff_ptr++;
		check_answer(false);
	}

	tms_command_nr(2,0x01,0);
return 0;
}

//////////////////////////////////////////////////////////////////////////////
//string syntax parcing functions
//////////////////////////////////////////////////////////////////////////////

//return hex number in range 0-0xf or error 0xff if char is not a hexadecimal digit
unsigned char char2hex(unsigned char c)
{
	unsigned char v;
	if(c>='0' && c<='9')
		v=c-'0';
	else
	if(c>='a' && c<='f')
		v=c-'a'+10;
	else
	if(c>='A' && c<='F')
		v=c-'A'+10;
	else
		v=0xff; //not a hex
	return v;
}

//skip leading space or tab chars in string
unsigned char* skip_space(unsigned char* ptr)
{
	while(1)
	{
		unsigned char c = *ptr;
		if( c==' ' || c==9 ) 
		{
			ptr++;
			continue;
		}
		else
			break;
	}
	return ptr;
}

//skip from string word which we expect to see
unsigned char* skip_expected_word(unsigned char* ptr, unsigned char* pword, unsigned int* presult)
{
	//assume error result
	*presult=0;
	ptr = skip_space(ptr);
	while(1)
	{
		unsigned char c1,c2;
		c2 = *pword;
		if(c2==0)
		{
			//end of comparing word
			*presult=1;
			return ptr;
		}
		c1 = *ptr;
		if(c1==c2)
		{
			//at this point strings are equal, but compare next chars
			ptr++;
			pword++;
			continue;
		}
		else
			break; //string has no expected word
	}
	return ptr;
}

//get word from string
unsigned char* get_and_skip_word(unsigned char* ptr, unsigned char* pword, unsigned int* presult)
{
	int i;
	unsigned char c;
	*presult=0;

	//assume error result
	*presult=0;
	ptr = skip_space(ptr);
	for(i=0; i<8; i++)
	{
		c = *ptr;
		if(c==' ' || c==9 || c==0 || c==0xd || c==0xa)
		{
			//end of getting word
			*pword=0;
			*presult=1;
			return ptr;
		}
		*pword++ = c;
		ptr++;
	}
	*pword=0;
	return ptr;
}

//read decimal integer from string
//returned 0xffffffff mean error
unsigned char* get_dec_integer(unsigned char* ptr, unsigned int* presult)
{
	unsigned int r=0;
	ptr=skip_space(ptr);
	while(1)
	{
		unsigned char c;
		c= *ptr;
		if(c>='0' && c<='9')
		{
			r=r*10+(c-'0');
			ptr++;
			continue;
		}
		else
		if(c==' ' || c==9)
		{
			*presult=r;
			break;
		}
		else
		{
			//unexpected char
			*presult=0xffffffff;
			break;
		}
	}
	return ptr;
}

unsigned char* read_hex_array(unsigned char* ptr, unsigned char* pdst, unsigned int num_bits, FILE* f, unsigned int* pcount, unsigned int* presult)
{
	unsigned char c,chr;
	unsigned int len,chr_count;
	char* pstr;

	//assume we fail
	*presult = 0;
	
	chr_count = 0;
	while(num_bits>0)
	{
		//get char from string and convert to hex digit
		chr = *ptr;
		c=char2hex(chr);
		if(c!=0xFF)
		{
			//hexadecimal digit is correct, save it
			*pdst++=c;
			//remember that 4 bits we got
			num_bits -= 4;
			//go to next char
			ptr++;
			chr_count++;
		}
		else
		{
			//char from string is not hexadecimal, but what is this?
			if(chr==')')
			{
				//end of hexadecimal array!!!
				ptr++;
				//return SUCCESS
				*presult = 1;
				*pcount=chr_count;
				return ptr;
			}
			else
			if(chr==0 || chr==0xd || chr==0xa)
			{
				//end of string, we need to continue by reading next string from file
				pstr = fgets(rbuffer,MAX_STRING_LENGTH-1,f);
				if(pstr==NULL)
				{
					//file read fails
					return NULL;
				}
				len = strlen(pstr);
				if(pstr[len-1]==0xA || pstr[len-1]==0xD)
					pstr[len-1] = 0;
				ptr = (unsigned char*)rbuffer;
				ptr = skip_space(ptr);
			}
			else
			{
				//unexpected char, error in syntax?
				printf("unexpected char %02X\n",chr);
				return NULL;
			}
		}
	}
	//get char from string and convert to hex digit
	chr = *ptr++;
	if(chr==')')
	{
		//yes, we see final bracket
		*presult = 1;
		*pcount=chr_count;
		return ptr;
	}
	return NULL;
}

/*
process typical strings:
SDR 16 TDI (FFFF) TDO (C0C7) MASK (FFFF);
SDR 16 TDI (FFFF) TDO (2027);
SDR 13 TDI (0000);
return 1 on success, 0 is fail
note that arguments can be very long and take several lines in a source file
*/
int do_SDR(FILE* f)
{
	int i,j,k;
	int has_tdi, has_tdo, has_mask, has_smask;
	unsigned int r;
	unsigned char b;
	unsigned char word[16];
	unsigned char* pdst;
	unsigned int num_bits = 0;
	unsigned int num_bytes;
	unsigned char* pdest;
	unsigned int* pdest_count;
	unsigned char* ptr=(unsigned char*)rbuffer;

	//at begin of string we expect to see word "SDR"
	ptr = skip_expected_word(ptr,(unsigned char*)"SDR",&r);
	if(r==0)
	{
		printf("syntax error for SDR command\n");
		return 0;
	}

	//now we expect to get decimal number of bits for shift
	ptr = get_dec_integer(ptr,&num_bits);
	if(num_bits==0xffffffff)
	{
		printf("syntax error for SDR command, cannot get number of bits\n");
		return 0;
	}

	//how many bytes? calculate space required and allocate
	num_bytes = (num_bits+7)/8;

	//each byte takes 2 chars in string, plus we reserve 8 additional bytes for safity
	if(alloc_sdr_data(num_bytes*2+8)==0)
	{
		printf("error on SDR command\n");
		return 0;
	}

	//we expect some words like TDI, TDO, MASK or SMASK here
	//order of words can be different
	has_tdi=0;
	has_tdo=0;
	has_mask=0;
	has_smask=0;
	while(1)
	{
		ptr = skip_space(ptr);
		ptr = get_and_skip_word(ptr,word,&r);
		if(r==0)
		{
			printf("syntax error for SDR command, cannot fetch parameter word\n");
			return 0;
		}
		
		//analyze words
		if(strcmp((char*)word,"TDI")==0)
		{
			has_tdi = 1;
			pdest = psdr_tdi_data;
			pdest_count  = &sdr_tdi_sz;
		}
		else
		if(strcmp((char*)word,"TDO")==0)
		{
			has_tdo = 1;
			pdest = psdr_tdo_data;
			pdest_count  = &sdr_tdo_sz;
		}
		else
		if(strcmp((char*)word,"MASK")==0)
		{
			has_mask = 1;
			pdest = psdr_mask_data;
			pdest_count  = &sdr_mask_sz;
		}
		else
		if(strcmp((char*)word,"SMASK")==0)
		{
			has_smask = 1;
			pdest = psdr_smask_data;
			pdest_count  = &sdr_smask_sz;
		}
		else
		if(strcmp((char*)word,";")==0)
		{
			//end of string!
			//send bitstream to jtag
			sdr_nbits(num_bits,has_tdo,has_mask,has_smask);
			break;
		}
		else
		{
			printf("syntax error for SDR command, unknown parameter word\n");
			return 0;
		}

		//parameter should be in parentheses
		ptr = skip_expected_word(ptr,(unsigned char*)"(",&r);
		if(r==0)
		{
			printf("syntax error for SDR command, expected char ( after TDI word\n");
			return 0;
		}

		//now expect to read hexadecimal array of tdi data
		ptr = read_hex_array(ptr,pdest,num_bits,f,pdest_count,&r);
	}
	return 1;
}

/*
process typical strings:
RUNTEST 53 TCK;
*/
void do_RUNTEST()
{
	//get command parameters
	unsigned int tck = 0;
	int n = sscanf(rbuffer,"RUNTEST %d TCK;",&tck);
	if(n==1)
	{
		//1 param
	}
	else
	{
		n = sscanf(rbuffer,"RUNTEST IDLE %d TCK",&tck);
		if(n==0)
		 printf("error processing RUNTEST\n");
	}
	if(tck)
		runidle(tck);
}

/*
process typical strings:
STATE IDLE;
*/
void do_STATE()
{
	Idle();
}

//reset JTAG
void do_TRST()
{
	goto_tlr();
}

void do_ENDDR()
{
}

void do_ENDIR()
{
}

void do_FREQUENCY()
{
	unsigned int clk_div;
	float freq = 1000000;
	float freq_achived;
	sscanf(rbuffer,"FREQUENCY %f",&freq);

	//iterate to find proper clock divider
	for(clk_div=0; clk_div<0x10000; clk_div++)
	{
		freq_achived = (float)30000000/(float)(clk_div+1);
		if(freq_achived<=freq)
		{
			printf("Frequency is set to %eHz (FTDI clk divider %04X), requred %eHz\n",freq_achived,clk_div,freq);

			//Command to set clock divisor
			byOutputBuffer[dwNumBytesToSend++] = 0x86;
			//Set 0xValueL of clock divisor
			byOutputBuffer[dwNumBytesToSend++] = clk_div & 0xFF;
			//Set 0xValueH of clock divisor
			byOutputBuffer[dwNumBytesToSend++] = (clk_div >> 8) & 0xFF;
			// Send off the clock divisor commands
			ftStatus = FT_Write_b(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
			return;
		}
	}

	//use default
	printf("Oops: %s\n",rbuffer);
	printf("WARNING: Frequency set was ignored, cannot recognize value!\n");
}

//////////////////////////////////////////////////////////////////////////////
// SVF PLAYER IS HERE
//////////////////////////////////////////////////////////////////////////////

int play_svf(FILE* f)
{
	//open connection to ftdi chip
	if(ftdi_init())
		return 1;

	// Wait for all the USB stuff to complete and work
	Sleep(50);

	configure_mpsse();

	//get_id();

	read_all();

	//read and process SVF file
	while(1)
	{
		int n,len;
		char* pstr;

		//get string from text file
		pstr = fgets(rbuffer,MAX_STRING_LENGTH-1,f);
		if(pstr==NULL)
			break;

		len = strlen(pstr);
		if(len>0)
		{
		    if(pstr[len-1]==0xA || pstr[len-1]==0xD)
			pstr[len-1] = 0;
		    if(len>1)
		    if(pstr[len-2]==0xA || pstr[len-2]==0xD)
			pstr[len-2] = 0;
		}

		//analyze 1st word
		n = sscanf(pstr,"%s",command);
		if(n==0)
			break;

		if(
			command[0]=='!' /* Altera Quartus marks SVF comment lines with '!' char */
			|| 
			(command[0]=='/' && command[1]=='/') /*Xilinx marks comment lines with '//' chars*/ 
			)
		{
			//line is commented
			if(check_answer(true)==0)
			{
				printf("error on check TDO answer\n");
				return 1;
			}

			//check important commented lines
			if( 
				(strcmp(command,"!CHECKING")==0) ||
				(strcmp(command,"!BULK")==0) ||
				(strcmp(command,"!PROGRAM")==0) ||
				(strcmp(command,"!VERIFY")==0) ||
				(command[0]=='/' && command[1]=='/')
				)
			{
				printf("-----------------------------------\n");
				printf("%s\n",rbuffer);
				line=0;
			}
		}
		else
		{
			line++;
			//real command is here
			if(strcmp(command,"SIR")==0)
				do_SIR();
			else
			if(strcmp(command,"SDR")==0)
				do_SDR(f);
			else
			if(strcmp(command,"RUNTEST")==0)
				do_RUNTEST();
			else
			if(strcmp(command,"STATE")==0)
				do_STATE();
			else
			if(strcmp(command,"TRST")==0)
				do_TRST();
			else
			if(strcmp(command,"ENDDR")==0)
				do_ENDDR();
			else
			if(strcmp(command,"ENDIR")==0)
				do_ENDIR();
			else
			if(strcmp(command,"FREQUENCY")==0)
				do_FREQUENCY();
			else
			if(strcmp(command,"HDR")==0)
			{}
			else
			if(strcmp(command,"TDR")==0)
			{}
			else
			if(strcmp(command,"HIR")==0)
			{}
			else
			if(strcmp(command,"TIR")==0)
			{}
			else
			{
				printf("Unknown command in line: %s\n",rbuffer);
				return 1;
			}
		}
	}

	flush();
	if(check_answer(true)==0)
	{
		printf("error on check TDO answer\n");
	}
	
	free_sdr_data();
	close_ftdi();
	return 0;
}



