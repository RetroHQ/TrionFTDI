#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ftdi.h"
#include "Types.h"

#pragma warning(disable:4302)

////////////////////////////////////////////////////////////////////////////////
// FT2232H
// Config Flash		JTAG
// Channel A		Channel B
// AD0, CCK			BD0, TCK
// AD1, CDI0		DB1, TDI
// AD2, CDI1		DB2, TDO
// AD3, SS#			DB3, TMS
// AD4, CRESET_N
// AD5, CDONE
////////////////////////////////////////////////////////////////////////////////

#define CA_CCK							0x01
#define CA_CDI0							0x02
#define CA_CDI1							0x04
#define CA_SS_N							0x08
#define CA_CRESET_N						0x10
#define CA_CDONE						0x20

#define CB_TCK							0x01
#define CB_TDI							0x02
#define CB_TDO							0x04
#define CB_TMS							0x08

#define FTDI_DEVICE						0x6010

////////////////////////////////////////////////////////////////////////////////
// EEPROM commands
////////////////////////////////////////////////////////////////////////////////

#define CMD_READ_STATUS_REGISTER1		0x05
#define CMD_READ_STATUS_REGISTER2		0x35
#define CMD_READ_DEVICE_ID				0x90
#define CMD_READ_UNIQUE_ID				0x4b
#define CMD_WRITE_STATUS_REGISTERS		0x01
#define CMD_WRITE_ENABLE				0x06
#define CMD_SECTOR_ERASE				0x20		// 4K sector
#define CMD_CHIP_ERASE					0x60
#define CMD_BLOCK_ERASE_32K				0x52
#define CMD_BLOCK_ERASE_64K				0xd8
#define CMD_PROGRAM_PAGE				0x02		// 256 byte page
#define CMD_READ_BYTES					0x03
#define CMD_WAKE_UP						0xab
#define CMD_RESET_ENABLE				0x66
#define CMD_RESET						0x99

#define	STATUS_IN_PROGRESS				0x01
#define	STATUS_WRITE_ENABLE				0x02
#define	STATUS_BLOCK_PROTECT_SHIFT		2
#define	STATUS_BLOCK_PROTECT_MASK		0x001f
#define	STATUS_REGISTER_PROTECT_SHIFT	7
#define	STATUS_REGISTER_PROTECT_MASK	0x0180
#define	STATUS_QUAD_ENABLE				0x0200

////////////////////////////////////////////////////////////////////////////////
// Terminate and free anything related to the device config
////////////////////////////////////////////////////////////////////////////////

ftdi_context* gFTDIA = 0;

void ConfigTerm()
{
	if (gFTDIA)
	{
		ftdi_usb_close(gFTDIA);
		ftdi_free(gFTDIA);
		gFTDIA = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Set config pins to idle (all in)
////////////////////////////////////////////////////////////////////////////////

bool ConfigIdle()
{
	// commands to set all inputs
	unsigned char buf[] =
	{
		SET_BITS_LOW,				// opcode: set low bits (ADBUS[0-7])
		CA_SS_N | CA_CRESET_N,		// pin states
		0							// pin direction, all input
	};
	const u32 buflen = sizeof(buf);

	// write the setup to the chip.
	return (ftdi_write_data(gFTDIA, buf, buflen) == buflen);
}

////////////////////////////////////////////////////////////////////////////////
// Set config pins to active
////////////////////////////////////////////////////////////////////////////////

bool ConfigControl(u8 nBits)
{
	// commands to set useful initial state and drive output pins
	unsigned char buf[] =
	{
		SET_BITS_LOW,									// opcode: set low bits (ADBUS[0-7])
		nBits,											// pin states
		CA_SS_N | CA_CRESET_N | CA_CDI0 | CA_CCK		// pin output, CRESET, SPI DO, CLK and SS
	};
	const u32 buflen = sizeof(buf);

	// write the setup to the chip.
	return (ftdi_write_data(gFTDIA, buf, buflen) == buflen);
}

////////////////////////////////////////////////////////////////////////////////
// Set CRESET_N / SS_N state
////////////////////////////////////////////////////////////////////////////////

u8 gGPIO = CA_CRESET_N | CA_SS_N;

bool FPGAReset(bool bReset)
{
	if (bReset) gGPIO &= ~CA_CRESET_N;
	else gGPIO |= CA_CRESET_N;
	return ConfigControl(gGPIO);
}

bool ConfigChipSelect(bool bSelect)
{
	if (bSelect) gGPIO &= ~CA_SS_N;
	else gGPIO |= CA_SS_N;
	return ConfigControl(gGPIO);
}

////////////////////////////////////////////////////////////////////////////////
// Initialise device config (config EEPROM, reset, etc...)
////////////////////////////////////////////////////////////////////////////////

#define SPI_6MHZ	9
#define SPI_7_5MHZ	8
#define SPI_10MHZ	5
#define SPI_12MHZ	4
#define SPI_15MHZ	3
#define SPI_20MHZ	2
#define SPI_30MHZ	1
#define SPI_60MHZ	0

////////////////////////////////////////////////////////////////////////////////

bool ConfigInit(u8 speed = SPI_10MHZ)
{
	s32 ret;
	
	// initialise FTDI lib
	if ((gFTDIA = ftdi_new()) == 0)
	{
		fprintf(stderr, "Unable to initialise libFTDI.\n");
		return false;
	}

	// set for interface A (config eeprom SPI) and open
	ftdi_set_interface(gFTDIA, INTERFACE_A);
	if ((ret = ftdi_usb_open(gFTDIA, 0x0403, FTDI_DEVICE)) < 0)
	{
		fprintf(stderr, "Unable to open FTDI device: %d (%s)\n", ret, ftdi_get_error_string(gFTDIA));
		ftdi_free(gFTDIA);
		gFTDIA = 0;
		return false;
	}

	// initialise MPSSE mode
	ftdi_usb_reset(gFTDIA);
	ftdi_set_bitmode(gFTDIA, 0, BITMODE_RESET);
	ftdi_set_bitmode(gFTDIA, 0, BITMODE_MPSSE);

	Sleep(50); // sleep 50 ms for setup to complete

	ConfigIdle();

	// setup SPI clocking etc...
	unsigned char buf[] =
	{
		TCK_DIVISOR,		// opcode: set clk divisor
		speed,				// argument: 60/(ARG+1)Mhz
		0x00,				// argument: high
		DIS_ADAPTIVE,		// opcode: disable adaptive clocking
		DIS_3_PHASE,		// opcode: disable 3-phase clocking
		SEND_IMMEDIATE
	};
	const u32 buflen = sizeof(buf);

	// write the clocking setup and set to idle
	if (!((ftdi_write_data(gFTDIA, buf, buflen) == buflen) && ConfigIdle()))
	{
		ConfigTerm();
		fprintf(stderr, "Unable to initalise FTDI device for config.\n");
		return false;
	}

	// all setup and not interfering with reset, etc...
	return true;
}

////////////////////////////////////////////////////////////////////////////////
// Write SPI data
////////////////////////////////////////////////////////////////////////////////

bool ConfigWriteSPI(const void *pOutData, const int nLength, void *pInData = 0)
{
	// initialise sending of data
	unsigned char buf[] =
	{
		(u8) (MPSSE_DO_WRITE | MPSSE_WRITE_NEG | (pInData != 0 ? MPSSE_DO_READ : 0)),
		(u8) (nLength - 1),
		(u8) ((nLength - 1) >> 8)
	};
	const u32 buflen = sizeof(buf);

	// start send
	bool bOk = ftdi_write_data(gFTDIA, buf, buflen) == buflen;

	// send data
	if (bOk)
	{
		// data supplied
		if (pOutData > (const void*)0xff)
		{
			bOk = ftdi_write_data(gFTDIA, (const unsigned char*)pOutData, nLength) == nLength;
		}
		// or write repeated character
		else
		{
			u8 byte = (u8)pOutData;
			u32 count = nLength;
			while (count-- && bOk)
			{
				bOk = ftdi_write_data(gFTDIA, &byte, 1) == 1;
			}
		}
	}

	// read if needed
	if (bOk && pInData) bOk = ftdi_read_data(gFTDIA, (unsigned char*)pInData, nLength) == nLength;

	return bOk;
}

////////////////////////////////////////////////////////////////////////////////
// Write single byte command over SPI
////////////////////////////////////////////////////////////////////////////////

bool ConfigWriteCommand(u8 cmd)
{
	const u32 buflen = 11;

	u8 buf[buflen];
	u8* ptr = buf;

	*ptr++ = SET_BITS_LOW;
	*ptr++ = gGPIO & ~CA_SS_N;
	*ptr++ = CA_SS_N | CA_CRESET_N | CA_CDI0 | CA_CCK;

	*ptr++ = (u8)(MPSSE_DO_WRITE | MPSSE_WRITE_NEG);
	*ptr++ = 0;
	*ptr++ = 0;

	*ptr++ = cmd;

	*ptr++ = SET_BITS_LOW;
	*ptr++ = gGPIO | CA_SS_N;
	*ptr++ = CA_SS_N | CA_CRESET_N | CA_CDI0 | CA_CCK;
	*ptr++ = SEND_IMMEDIATE;

	return ftdi_write_data(gFTDIA, buf, buflen) == buflen;
}

////////////////////////////////////////////////////////////////////////////////
// Write command with address and return data of given size
////////////////////////////////////////////////////////////////////////////////

bool ConfigWriteCommandWithAddrAndData(u8 cmd, u32 addr, const void *bufOut, void *bufIn, u32 size)
{
	const u32 buflen = 14 + ((size > 0) ? (size + 3) : 0);
	u8* buf = new u8[buflen];
	u8* ptr = buf;
	  
	*ptr++ = SET_BITS_LOW;
	*ptr++ = gGPIO & ~CA_SS_N;
	*ptr++ = CA_SS_N | CA_CRESET_N | CA_CDI0 | CA_CCK;

	*ptr++ = (u8)(MPSSE_DO_WRITE | MPSSE_WRITE_NEG);
	*ptr++ = 4 - 1;
	*ptr++ = 0;

	*ptr++ = cmd;
	*ptr++ = addr >> 16;
	*ptr++ = addr >> 8;
	*ptr++ = addr;
	
	if (size)
	{
		*ptr++ = (u8)(MPSSE_DO_WRITE | MPSSE_WRITE_NEG | (bufIn != 0 ? MPSSE_DO_READ : 0));
		*ptr++ = (u8)(size - 1);
		*ptr++ = (u8)((size - 1) >> 8);

		if (bufOut > (void*)0xff) memcpy(ptr, bufOut, size);
		else memset(ptr, (u8)bufOut, size);
		ptr += size;
	}

	*ptr++ = SET_BITS_LOW;
	*ptr++ = gGPIO | CA_SS_N;
	*ptr++ = CA_SS_N | CA_CRESET_N | CA_CDI0 | CA_CCK;
	*ptr++ = SEND_IMMEDIATE;

	// write the command stream
	bool bOk = ftdi_write_data(gFTDIA, buf, buflen) == buflen;

	// if we need to read data back as well, do it now
	if (bOk && bufIn)
	{
		bOk = ftdi_read_data(gFTDIA, (unsigned char*)bufIn, size) == size;
	}

	delete buf;
	return bOk;
}

////////////////////////////////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////////////////////////////////

bool ConfigReset()
{
	return ConfigWriteCommand(CMD_RESET_ENABLE) && ConfigWriteCommand(CMD_RESET);
}

////////////////////////////////////////////////////////////////////////////////
//
// Wake up if the EEPROM has been put to sleep (Trion will do this after config)
//
////////////////////////////////////////////////////////////////////////////////

bool ConfigWakeUp()
{
	return ConfigWriteCommand(CMD_WAKE_UP);
}

////////////////////////////////////////////////////////////////////////////////
// Read config device id
////////////////////////////////////////////////////////////////////////////////

bool ConfigReadDeviceId(u16 *id)
{
	return ConfigWriteCommandWithAddrAndData(CMD_READ_DEVICE_ID, 0, 0, id, 2);
}

////////////////////////////////////////////////////////////////////////////////
// Read config device id
////////////////////////////////////////////////////////////////////////////////

bool ConfigReadUniqueId(void *uid)
{
	return ConfigWriteCommandWithAddrAndData(CMD_READ_UNIQUE_ID, 0, 0, uid, 16);
}

////////////////////////////////////////////////////////////////////////////////
// Enable write to chip
////////////////////////////////////////////////////////////////////////////////

bool ConfigWriteEnable()
{
	return ConfigWriteCommand(CMD_WRITE_ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
// Poll until operation is complete
////////////////////////////////////////////////////////////////////////////////

bool ConfigPollStatusComplete()
{
	bool bOk = false;

	if (	ConfigChipSelect(true) &&
			ConfigWriteSPI((void*)CMD_READ_STATUS_REGISTER1, 1, 0))
	{
		u32 timeout = 20;
		u8 status;
		do
		{
			// wait a bit to stop spamming libFTDI (it doesnt like it)
			Sleep(1);
			bOk = ConfigWriteSPI(0, 1, &status);
		}
		while (--timeout && bOk & (status & STATUS_IN_PROGRESS));
		
		bOk &= ConfigChipSelect(false) && timeout != 0;
	}

	return bOk;
}

////////////////////////////////////////////////////////////////////////////////
// Erase whole chip
////////////////////////////////////////////////////////////////////////////////

bool ConfigEraseAll()
{
	return	ConfigWriteEnable() &&
			ConfigWriteCommand(CMD_CHIP_ERASE) &&
			ConfigPollStatusComplete();
}

////////////////////////////////////////////////////////////////////////////////
// Erase sector (4K)
////////////////////////////////////////////////////////////////////////////////

bool ConfigEraseSector(u32 addr)
{
	return	ConfigWriteEnable() &&
			ConfigWriteCommandWithAddrAndData(CMD_SECTOR_ERASE, addr, 0, 0, 0) &&
			ConfigPollStatusComplete();
}

////////////////////////////////////////////////////////////////////////////////
// Erase block (32K/64K)
////////////////////////////////////////////////////////////////////////////////

bool ConfigEraseBlock32(u32 addr)
{
	return	ConfigWriteEnable() &&
			ConfigWriteCommandWithAddrAndData(CMD_BLOCK_ERASE_32K, addr, 0, 0, 0) &&
			ConfigPollStatusComplete();
}

bool ConfigEraseBlock64(u32 addr)
{
	return	ConfigWriteEnable() &&
			ConfigWriteCommandWithAddrAndData(CMD_BLOCK_ERASE_64K, addr, 0, 0, 0) &&
			ConfigPollStatusComplete();
}

////////////////////////////////////////////////////////////////////////////////
// Erase given area
////////////////////////////////////////////////////////////////////////////////

bool ConfigEraseArea(u32 addr, u32 size)
{
	bool bOk = true;
	
	// align start address and size to 4K sectors (minimum erase size)
	size += (4096 - (addr & 4095)) & 4095;
	addr &= ~4095;
	size = (size + 4095) & ~4095;

	while (bOk && size)
	{
		// aligned to 64K
		if (((addr & 65535) == 0) && (size >= 65536))
		{
			bOk = ConfigEraseBlock64(addr);
			addr += 65536;
			size -= 65536;
		}
		else if (((addr & 32767) == 0) && (size >= 32768))
		{
			bOk = ConfigEraseBlock32(addr);
			addr += 32768;
			size -= 32768;
		}
		else
		{
			bOk = ConfigEraseSector(addr);
			addr += 4096;
			size -= 4096;
		}
	}

	return bOk;
}

////////////////////////////////////////////////////////////////////////////////
// Write page (max 256 bytes), data will wrap within 256 byte page, so max
// sequential write is 256-(addr&255).
////////////////////////////////////////////////////////////////////////////////

bool ConfigWritePage(u32 nAddress, const void* pData, u32 nSize = 256)
{
	if (nSize > 256)
	{
		return false;
	}

	return	ConfigWriteEnable() &&
			ConfigWriteCommandWithAddrAndData(CMD_PROGRAM_PAGE, nAddress, pData, 0, nSize) &&
			ConfigPollStatusComplete();
}

////////////////////////////////////////////////////////////////////////////////
// Read bytes starting at address given
////////////////////////////////////////////////////////////////////////////////

bool ConfigReadBytes(u32 nAddress, void* pData, u32 nSize = 256)
{
	return	ConfigWriteCommandWithAddrAndData(CMD_READ_BYTES, nAddress, 0, pData, nSize);
}

////////////////////////////////////////////////////////////////////////////////
// Show info on connected EEPROM
////////////////////////////////////////////////////////////////////////////////

void ShowDeviceInfo()
{
	u16 id = 0xffff;
	ConfigReadDeviceId(&id);
	const char* pDev = "Unknown";
	switch (id)
	{
	case 0x13c8:
		pDev = "GigaDevices GD25Q80E";
		break;
	}

	fprintf(stdout, "Config manufacturer / device ID %04X (%s)\n", id, pDev);

	u8 uid[16];
	ConfigReadUniqueId(&uid);
	fprintf(stdout, "Config unique ID ");
	for (u32 n = 0; n < 16; n++)
	{
		fprintf(stdout, "%02X", uid[n]);
	}
	fprintf(stdout, "\n");

}

////////////////////////////////////////////////////////////////////////////////
// Parse hex file to get size and validity
////////////////////////////////////////////////////////////////////////////////

s32 HexGetSize(const char* pFilename)
{
	s32 nFileSize = -1;
	FILE* f;
	if (fopen_s(&f, pFilename, "rt") == 0)
	{
		nFileSize = 0;
		// scan and see how big it is in actual binary terms
		while (1)
		{
			char c = fgetc(f);
			if (feof(f))
			{
				break;
			}

			// lowercase
			c = tolower(c);

			// see if character is valid hex
			if ((c >= 'a' && c <= 'f') ||
				(c >= '0' && c <= '9'))
			{
				nFileSize++;
			}

			// on an invalid character (non-hex and non-whitespace) return error
			else if (c != 9 && c != ' ' && c != 10 && c != 13 && c != 0)
			{
				nFileSize = -1;
				break;
			}
		}

		fclose(f);
	}

	return nFileSize >> 1;
}

////////////////////////////////////////////////////////////////////////////////
// Get number of bytes of binary from hex file
////////////////////////////////////////////////////////////////////////////////

u32 HexGetBytes(void* buf, FILE* f, u32 size)
{
	u8 data = 0;
	u32 nibbles = 0;
	u8* out = (u8*)buf;
	
	while (size)
	{
		char c = fgetc(f);
		if (feof(f))
		{
			break;
		}

		// lowercase
		c = tolower(c);

		// get nibble from character
		bool added = true;
		if (c >= 'a' && c <= 'f')
		{
			data <<= 4;
			data |= c - ('a' - 10);
		}
		else if (c >= '0' && c <= '9')
		{
			data <<= 4;
			data |= c - '0';
		}
		else
		{
			added = false;
		}

		// if a whole byte has been read, write it out
		if (added && (nibbles++ & 1))
		{
			*out++ = data;
			size--;
		}
	}

	return nibbles >> 1;
}

////////////////////////////////////////////////////////////////////////////////
// Program hex file
// Optionally erase only the blocks affected and also verify once complete
////////////////////////////////////////////////////////////////////////////////

#define PROG_ERASE			1
#define PROG_PROGRAM		2
#define PROG_VERIFY			4

////////////////////////////////////////////////////////////////////////////////

void ConfigProgramHex(const char* pFilename, const u32 writeAddr, const u8 mode = PROG_PROGRAM | PROG_ERASE | PROG_VERIFY)
{
	s32 hexSize = HexGetSize(pFilename);
	if (hexSize < 0) printf("Hex file corrupt (%s).\n", pFilename);
	else
	{
		for (u32 n = 0; n < 3; n++)
		{
			if (mode & (1 << n))
			{
				if (n == 0)
				{
					printf("Erasing... ");
					if (ConfigEraseArea(writeAddr, hexSize)) printf("OK!\n");
					else
					{
						printf("FAILED!\n");
						break;
					}
				}
				else
				{
					if (n == 1) printf("Programming... ");
					else printf("Verifying... ");

					s32 percent = -1;
					u32 addr = writeAddr;
					FILE* f;
					if (fopen_s(&f, pFilename, "rt") == 0)
					{
						u32 totalOut = 0;
						while (totalOut < (u32) hexSize)
						{
							// read in the next page
							u8 buf[256];
							u32 read = HexGetBytes(buf, f, 256);

							// verify
							if (n == 2)
							{
								u8 buf2[256];
								if (!ConfigReadBytes(addr, buf2, read) || memcmp(buf, buf2, read) != 0)
								{
									break;
								}
							}

							// program
							else
							{
								// write it to the config prom
								if (!ConfigWritePage(addr, buf, read))
								{
									break;
								}
							}

							// update progress
							s32 npercent = (totalOut * 100) / hexSize;
							if (npercent != percent)
							{
								percent = npercent;
								printf("%02d%%\b\b\b", percent);
							}

							addr += read;
							totalOut += read;
						}
						fclose(f);

						// report how we did
						if (totalOut == hexSize)
						{
							printf("OK!\n");
						}
						else
						{
							printf("FAILED!\n");
						}
					}
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// CLI main
////////////////////////////////////////////////////////////////////////////////

u32 StringToNumber(const char* opt)
{
	int base = 10;
	if (*opt == '$')
	{
		opt++;
		base = 16;
	}
	else if (opt[0] == '0' && opt[1] == 'x')
	{
		opt += 2;
		base = 16;
	}
	return (u32)strtol(opt, NULL, base);
}

int main(int argc, const char **argv)
{
	const char* pFilename = "testbed.hex";

	u8 nSPIFreq = SPI_20MHZ;

	if (argc == 1)
	{
		printf("Usage: %s [commands]\n\n"
			"Commands:\n"
			"-f {freq}                 Set SPI frequency (Mhz), default 20Mhz\n"
			"-i                        Display chip information\n"
			"-c                        Trigger FPGA config\n"
			"-e                        Erase whole chip\n"
			"-w[ev] file.hex [addr]    Write hex file with optial [e]rase and [v]erify to address (default 0, use $ or 0x for hex)\n"
			"-v file.hex [addr]        Verify contents of config prom at address match this file\n"
			, argv[0]);
	}
	
	// first do a pass to get the SPI frequency for initialisation
	for (s32 n = 1; n < argc; n++)
	{
		if (_stricmp(argv[n], "-f") == 0)
		{
			n++;
			if (n < argc)
			{
				float freq = (float) atof(argv[n]);
				if (freq > 60) freq = 60;
				if (freq < 6) freq = 6;
				nSPIFreq = (u32) ceil(60.f / freq) - 1;
 			}
		}
	}

	// initialise config programming
	if (ConfigInit(nSPIFreq))
	{
		// wake up and reset the chip incase it has been powered down
		ConfigWakeUp();
		ConfigReset();
		
		// process other commands in order
		for (s32 n = 1; n < argc; n++)
		{
			// info
			if (_stricmp(argv[n], "-i") == 0)
			{
				printf("SPI frequency %dMhz\n", 60 / (nSPIFreq + 1));
				ShowDeviceInfo();
			}

			// FPGA config
			else if (_stricmp(argv[n], "-c") == 0)
			{
				FPGAReset(true);
				Sleep(50);
				ConfigWakeUp();
				ConfigReset();
			}

			// erase chip
			else if (_stricmp(argv[n], "-e") == 0)
			{
				printf("Erasing... ");
				if (ConfigEraseAll()) printf("OK!\n");
				else printf("FAILED!\n");
			}

			// write hex
			else if (_strnicmp(argv[n], "-w", 2) == 0 || _stricmp(argv[n], "-v") == 0)
			{
				u8 param = PROG_PROGRAM;
				if (_stricmp(argv[n], "-v") == 0)
				{
					param = PROG_VERIFY;
				}
				else
				{
					char c;
					const char* opt = argv[n] + 2;
					while ((c = *opt++))
					{
						c = tolower(c);
						if (c == 'e') param |= PROG_ERASE;
						else if (c == 'v') param |= PROG_VERIFY;
					}
				}

				n++;
				if (n < argc)
				{
					const char* pFilename = argv[n];
					u32 addr = 0;

					if (((n + 1) < argc) && argv[n + 1][0] != '-')
					{
						n++;
						addr = StringToNumber(argv[n]);
					}

					ConfigProgramHex(pFilename, addr, param);
				}
				else
				{
					printf("Error: No filename specified.\n");
				}

			}
		}

		// make sure we leave with all signals inactive
		ConfigIdle();
		ConfigTerm();
	}
 }
