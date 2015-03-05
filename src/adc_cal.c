/* ------------------------------------------------------------------------
File   : adc_cal.c

Descr  : Functions to make the ADC calibration constants -which are stored
         in EEPROM at production (ELMB128)- accessible.

History: 10JAN.03; Henk B&B; Definition.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc_cal.h"
#include "can.h"
#include "crc.h"
#include "eeprom.h"
#include "objects.h"
#include "store.h"

/* ------------------------------------------------------------------------ */
/* Globals */

/* Storage space for error bits concerning the ADC */
static BYTE AdcError = 0;

/* To enable a single write to an calibration constant location in EEPROM */
static BOOL AdcCalibConstWriteEnabled = FALSE;

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BOOL adc_valid_calib_const( BYTE od_range_id );

/* ------------------------------------------------------------------------ */

BOOL adc_set_calib_const( BYTE od_range_id, BYTE index, BYTE *val )
{
  /* Writes one of the calibration constants of the calibration parameter set
     for one particular voltage range.
     The new value is stored directly in EEPROM as part of the set
     and also the CRC of the parameter datablock is updated.
     This function is used for permanently storing the ADC calibration
     constants (or restoring lost/corrupted constants), and thus
     should be used with care. */
  BYTE   result = TRUE;
  BYTE   byt;
  UINT16 ee_addr;
  UINT16 crc;

  /* Has the write operation been enabled ? */
  if( (AdcCalibConstWriteEnabled & TRUE) == FALSE ) return FALSE;

  /* Allow 1 write operation at a time */
  AdcCalibConstWriteEnabled = FALSE;

  /* EEPROM address for this constant */
  ee_addr = (STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
	     index*STORE_ADC_CALIB_PARSIZE);

  /* Store the constant in EEPROM and check, byte-by-byte */
  for( byt=0; byt<STORE_ADC_CALIB_PARSIZE; ++byt, ++ee_addr )
    {
      BYTE val_b;
      val_b = val[byt];
      if( eepromw_read(ee_addr) != val_b ) eepromw_write( ee_addr, val_b );
      if( eepromw_read(ee_addr) != val_b ) result = FALSE; /* Check */
    }

  /* (Re)calculate CRC of the full set of constants */
  ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;
  crc = crc16_eeprom( ee_addr, STORE_ADC_CALIB_SIZE );

  /* Store CRC immediately behind the data, MSB first */
  byt = (BYTE) ((crc & 0xFF00) >> 8);
  ee_addr += STORE_ADC_CALIB_SIZE;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */
  /* And now the CRC LSB */
  byt = (BYTE) (crc & 0x00FF);
  ++ee_addr;
  eepromw_write( ee_addr, byt );
  if( eepromw_read(ee_addr) != byt ) result = FALSE; /* Check */

  /* The 'valid data' byte */
  ++ee_addr;
  if( result == TRUE )
    {
      /* Now we have a valid calibration constants datablock ! */
      if( eepromw_read(ee_addr) != STORE_VALID_CHAR )
	eepromw_write( ee_addr, STORE_VALID_CHAR );
      if( eepromw_read(ee_addr) != STORE_VALID_CHAR ) /* Check */
	result = FALSE;
    }
  else
    {
      /* Mark it as an invalid parameter block */
      eepromw_write( ee_addr, 0xFF );
    }

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_CALIB_CNST;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS,
			   STORE_ADC_CALIB, STORE_ADC_CALIB_SIZE,
			   0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_get_calib_const( BYTE od_range_id, BYTE index, BYTE *val,
			  BOOL send_emergency )
{
  BYTE   byt, result = TRUE;
  UINT16 ee_addr;

  AdcCalibConstWriteEnabled = FALSE;

  /* Initialize all bytes: assume there are 4 */
  for( byt=0; byt<4; ++byt ) val[byt] = 0;

  /* Is there a valid calibration constants datablock
     for the designated voltage range? */
  if( adc_valid_calib_const( od_range_id ) )
    {
      /* Start address of this set of constants */
      ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;

      /* Check the CRC (run the CRC on the datablock plus
	 the stored CRC value: the result should be zero) */
      if( crc16_eeprom( ee_addr, STORE_ADC_CALIB_SIZE+2 ) == 0 )
	{
	  /* EEPROM address for this constant */
	  ee_addr = (STORE_ADC_CALIB_ADDR +
		     od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
		     index*STORE_ADC_CALIB_PARSIZE);

	  /* Copy the constant value to val[], byte-by-byte */
	  for( byt=0; byt<STORE_ADC_CALIB_PARSIZE; ++byt, ++ee_addr )
	    {
	      val[byt] = eepromw_read( ee_addr );
	    }
	}
      else
	{
	  /* Error in CRC */
	  AdcError |= ADC_ERR_CALIB_CNST;

	  if( send_emergency )
	    {
	      /* CANopen Error Code 0x5000: device hardware */
	      can_write_emergency( 0x00, 0x50, EMG_EEPROM_READ_PARS,
				   STORE_ADC_CALIB, STORE_ERR_CRC,
				   0, ERRREG_MANUFACTURER );
	    }
	  result = FALSE;
	}
    }
  else
    {
      /* No valid constants available */
      result = FALSE;
    }
  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_erase_calib_const( BYTE od_range_id, BYTE val )
{
  /* Erases a complete set of calibration constants for
     one particular voltage range from EEPROM. To be used with care.... */
  BYTE   byt, result = TRUE;
  UINT16 ee_addr;

  /* Has the erase operation been enabled ? */
  if( (AdcCalibConstWriteEnabled & TRUE) == FALSE ) return FALSE;

  /* Allow 1 erase operation at a time */
  AdcCalibConstWriteEnabled = FALSE;

  /* Allow only when 'val' has a particular value */
  if( val != 0xEE ) return FALSE;

  /* EEPROM address for this set of constants */
  ee_addr = STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE;

  /* Erase, byte-by-byte, by setting EEPROM locations to 0xFF */
  for( byt=0; byt<STORE_ADC_CALIB_BLOCKSIZE; ++byt, ++ee_addr )
    {
      if( eepromw_read(ee_addr) != 0xFF ) eepromw_write( ee_addr, 0xFF );
      if( eepromw_read(ee_addr) != 0xFF ) result = FALSE; /* Check */
    }

  if( result == FALSE )
    {
      AdcError |= ADC_ERR_CALIB_CNST;

      /* CANopen Error Code 0x5000: device hardware */
      can_write_emergency( 0x00, 0x50, EMG_EEPROM_WRITE_PARS,
			   STORE_ADC_CALIB, STORE_ADC_CALIB_BLOCKSIZE,
			   0, ERRREG_MANUFACTURER );
    }

  return result;
}

/* ------------------------------------------------------------------------ */

BOOL adc_calib_const_write_enable( BYTE val )
{
  /* Set the boolean only when 'val' has a particular value */
  AdcCalibConstWriteEnabled = FALSE;
  if( val == 0xA5 )
    {
      /* Enable a single write to a calibration constant location in EEPROM */
      AdcCalibConstWriteEnabled = TRUE;
    }
  return AdcCalibConstWriteEnabled;
}

/* ------------------------------------------------------------------------ */

static BOOL adc_valid_calib_const( BYTE od_range_id )
{
  /* Is the calibration constants datablock valid for this voltage range ? */
  UINT16 ee_addr;

  /* Address of the 'valid' byte */
  ee_addr = (STORE_ADC_CALIB_ADDR + od_range_id*STORE_ADC_CALIB_BLOCKSIZE +
	     STORE_ADC_CALIB_PARS*STORE_ADC_CALIB_PARSIZE + 2);

  return( eepromw_read(ee_addr) == STORE_VALID_CHAR );
}

/* ------------------------------------------------------------------------ */
