/* ------------------------------------------------------------------------
File   : store.h

Descr  : Definitions and declarations of EEPROM parameter storage routines.

History: 21SEP.00; Henk B&B; Definition.
	 23JUL.01;  "    " ; Added PDO stuff;
	                     increased STORE_INFO_SIZE from 3 to 4;
			     decreased STORE_BLOCK_SIZE from 32 to 16;
			     rearranged variable storage.
	 24JUL.01;  "    " ; Go from WORD to BYTE addresses, which is alright
	                     upto 256 bytes of EEPROM storage...
	 31OCT.01;  "    " ; Don't use EEPROM address 0.
--------------------------------------------------------------------------- */

#ifndef STORE_H
#define STORE_H

/* The number of individual data storage blocks */
#define STORE_BLOCK_CNT                 8

/* Maximum size of a data block (plus length word), in bytes (16) */
#define STORE_BLOCK_SIZE                0x10

/* Size of the info on a data block, in bytes (3 in use, 1 unused) */
#define STORE_INFO_SIZE                 4

/* Byte that signals the presence of a valid stored data block */
#define STORE_VALID_CHAR                'V'

/* ------------------------------------------------------------------------ */
/* EEPROM indices and addresses */

/* Parameter- and info-block indices */
#define STORE_TPDO                      0
#define STORE_RPDO                      1
#define STORE_GUARDING                  2
#define STORE_CAN                       3
#define STORE_APP                       4

/* Other */
#define STORE_ADC_CALIB                 0xFE
#define STORE_ELMB_SN                   0xFF

/* EEPROM address offset for info blocks */
#define STORE_INFO_ADDR                 0x01

/* EEPROM address offset for data blocks, stored behind the info blocks */
#define STORE_DATA_ADDR                 (STORE_INFO_ADDR + \
                                         STORE_BLOCK_CNT*STORE_INFO_SIZE)

/* EEPROM address offset for (more radiation-tolerant) variable storage */
#define STORE_VAR_ADDR                  (STORE_DATA_ADDR + \
                                         STORE_BLOCK_CNT*STORE_BLOCK_SIZE)

/* Using the above constants STORE_VAR_ADDR = 1 + 8*4 + 8*16 = 161 = 0xA1,
   which means there are still up to 95 = 0x5F EEPROM locations (bytes)
   available for the stuff shown below */

/* ------------------------------------------------------------------------ */
/* EEPROM variable storage:
   (Global) variables that do not change very often are stored in EEPROM
   after initialisation and reread from EEPROM before every use;
   this makes the application more radiation-tolerant */

/* CAN stuff */
#define EE_NODEID                       (STORE_VAR_ADDR + 0x00)
#define EE_RTRIDHI                      (STORE_VAR_ADDR + 0x01)
#define EE_RTRIDLO                      (STORE_VAR_ADDR + 0x02)
#define EE_RTR_DISABLED                 (STORE_VAR_ADDR + 0x03)
#define EE_CANOPEN_OPSTATE_INIT         (STORE_VAR_ADDR + 0x04)
#define EE_CAN_BUSOFF_MAXCNT            (STORE_VAR_ADDR + 0x05)

/* Guarding stuff */
#define EE_LIFETIMEFACTOR               (STORE_VAR_ADDR + 0x08)
#define EE_HEARTBEATTIME                (STORE_VAR_ADDR + 0x09)

/* PDO stuff (reserve enough space for the settings of multiple PDOs, upto 8)*/
#define EE_PDO_MAX                      8
#define EE_PDO_TTYPE                    (STORE_VAR_ADDR + 0x10)
#define EE_PDO_ETIMER_LO                (EE_PDO_TTYPE     + EE_PDO_MAX)
#define EE_PDO_ETIMER_HI                (EE_PDO_ETIMER_LO + EE_PDO_MAX)
#define EE_TPDO_ONTIMER                 (EE_PDO_ETIMER_HI + EE_PDO_MAX)

/* User application stuff */
#define EE_APP_CHANS                    (STORE_VAR_ADDR + 0x30)
#define EE_APP_SOMETHING                (STORE_VAR_ADDR + 0x31)
/* ...etc...etc....etc........ */

/* ------------------------------------------------------------------------ */
/* EEPROM storage for addresses 256 and up */

/* Storage for an ELMB serial number */
/* ================================= */

/* Sizes */
#define STORE_ELMB_SN_SIZE              4

/* Location where the ELMB Serial Number is stored (there is no copy in RAM).
   The serial number is followed by a 2-byte CRC and a 'valid' byte */
#define STORE_ELMB_SN_ADDR              0x100
#define STORE_ELMB_SN_VALID_ADDR      (STORE_ELMB_SN_ADDR+STORE_ELMB_SN_SIZE+2)

/* Storage for calibration constants */
/* ================================= */
/* There are 6 voltage ranges for which calibration constants
   are stored (no dependency on wordrate or polarity).
   We store 4 gain factors and reserve space for 4 more factors
   or constants (in case the offset will be used in the calibration,
   in the future), as well as space for 1 calibration configuration parameter
   (could contain e.g. the reference voltage used in the calibration,
    but this is undefined for now),
   Three bytes are assigned to each factor or constant.
   Each set of constants is followed by a 2-byte CRC and a 'valid' byte
   (and 2 stuff bytes to get to a 'nice' parameter blocksize of 32) */

/* Location where the ADC calibration constants are stored
   (there should be space for 6x STORE_ADC_CALIB_BLOCKSIZE sized datablocks) */
#define STORE_ADC_CALIB_ADDR            0x120

/* Sizes: STORE_ADC_CALIB_SIZE is the size of the actual meaningful data
   excluding CRC (2 bytes) and 'valid'-byte (1 byte);
   STORE_ADC_CALIB_BLOCKSIZE is the datablock size reserved for it
   including CRC and 'valid'-byte */
#define STORE_ADC_CALIB_PARSIZE         3
#define STORE_ADC_CALIB_PARS            9
#define STORE_ADC_CALIB_SIZE            (STORE_ADC_CALIB_PARS*\
                                         STORE_ADC_CALIB_PARSIZE)
#define STORE_ADC_CALIB_BLOCKSIZE       (STORE_ADC_CALIB_SIZE+2+1+2)
#define STORE_ADC_CALIB_BLOCKS          6

/* ------------------------------------------------------------------------ */
/* Error IDs */

#define STORE_OKAY                      0x00
#define STORE_ERR_CRC                   0x01
#define STORE_ERR_LENGTH                0x02
#define STORE_ERR_INFO                  0x04

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BOOL storage_save_parameters  ( BYTE od_subindex );
BOOL storage_set_defaults     ( BYTE od_subindex );
void storage_check_load_status( void );
BOOL storage_write_block      ( BYTE storage_index,
				BYTE size,
				BYTE *block );
BOOL storage_read_block       ( BYTE storage_index,
				BYTE expected_size,
				BYTE *block );

#endif /* STORE_H */
/* ------------------------------------------------------------------------ */
