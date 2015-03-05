/* ------------------------------------------------------------------------
File   : adc_cal.h

Descr  : Declarations of ELMB calibration constants access functions.

History: 10JAN.03; Henk B&B; Definition; contains functions to make the
                             ADC calibration constants, which are stored
			     in EEPROM at production (ELMB128), accessible
--------------------------------------------------------------------------- */

#ifndef ADC_CAL_H
#define ADC_CAL_H

/* ------------------------------------------------------------------------ */
/* Error ID bits */

/* In AdcError status byte */
#define ADC_ERR_CALIB_CNST          0x08

/* ------------------------------------------------------------------------ */
/* Function prototypes */

BOOL   adc_set_calib_const         ( BYTE od_range_id, BYTE index, BYTE *val );
BOOL   adc_get_calib_const         ( BYTE od_range_id, BYTE index, BYTE *val,
				     BOOL send_emergency );
BOOL   adc_erase_calib_const       ( BYTE od_range_id, BYTE val );
BOOL   adc_calib_const_write_enable( BYTE val );

/* ------------------------------------------------------------------------ */
#endif /* ADC_CAL_H */
