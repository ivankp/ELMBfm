/* ------------------------------------------------------------------------
File   : objects.h

Descr  : Definitions of the CANopen Object Dictionary objects
         for this application.

History: --JUL.00; Henk B&B; First version.
--------------------------------------------------------------------------- */

#ifndef OBJECTS_H
#define OBJECTS_H

/* ------------------------------------------------------------------------ */

#define OD_NO_OF_ENTRIES        0		/* Often used for subindex 0 */

/* ============================================================== */
/* Communication Profile objects */

#define OD_DEVICE_INFO_HI       0x10		/* Objects 0x10.. */
#define OD_DEVICE_TYPE_LO       0x00		/* Object  0x1000 */
#define OD_ERROR_REG_LO         0x01		/* Object  0x1001 */
#define OD_STATUS_REG_LO        0x02		/* Object  0x1002 */
#define OD_DEVICE_NAME_LO       0x08		/* Object  0x1008 */
#define OD_HW_VERSION_LO        0x09		/* Object  0x1009 */
#define OD_SW_VERSION_LO        0x0A		/* Object  0x100A */
#define OD_GUARDTIME_LO         0x0C		/* Object  0x100C */
#define OD_LIFETIME_FACTOR_LO   0x0D		/* Object  0x100D */
#define OD_STORE_PARAMETERS_LO  0x10		/* Object  0x1010 */
#define OD_DFLT_PARAMETERS_LO   0x11		/* Object  0x1011 */
#define OD_HEARTBEAT_TIME_LO    0x17		/* Object  0x1017 */
#define OD_IDENTITY_LO          0x18		/* Object  0x1018 */
#define OD_STORE_ALL            1
#define OD_STORE_COMM_PARS      2
#define OD_STORE_APP_PARS       3

#define OD_RPDO_PAR_HI          0x14		/* Objects 0x14.. */
#define OD_RPDO1_PAR_LO         0x00		/* Object  0x1400 */
#define OD_RPDO2_PAR_LO         0x01		/* Object  0x1401 */

#define OD_RPDO_MAP_HI          0x16		/* Objects 0x16.. */
#define OD_RPDO1_MAP_LO         0x00		/* Object  0x1600 */
#define OD_RPDO2_MAP_LO         0x01		/* Object  0x1601 */

#define OD_TPDO_PAR_HI          0x18		/* Objects 0x18.. */
#define OD_TPDO1_PAR_LO         0x00		/* Object  0x1800 */
#define OD_TPDO2_PAR_LO         0x01		/* Object  0x1801 */
#define OD_PDO_COBID            1
#define OD_PDO_TRANSMTYPE       2
#define OD_PDO_INHIBITTIME      3
#define OD_PDO_DUMMY_ENTRY      4
#define OD_PDO_EVENT_TIMER      5

#define OD_TPDO_MAP_HI          0x1A		/* Objects 0x1A.. */
#define OD_TPDO1_MAP_LO         0x00		/* Object  0x1A00 */
#define OD_TPDO2_MAP_LO         0x01		/* Object  0x1A01 */

/* ============================================================== */
/* Manufacturer-specific objects */

/* Application-specific objects */
#define OD_APP_HI               0x20		/* Objects 0x20.. */
#define OD_APP_LO               0x00		/* Object  0x2000 */
#define OD_APP_ARR_HI           0x21		/* Objects 0x21.. */
#define OD_APP_ARR_LO           0x00		/* Object  0x2100 */
/* etc., etc., etc. */

/* Analog inputs calibration stuff */
#define OD_ADC_CALIB_PARS_HI    0x2B		/* Objects 0x2B.. */
#define OD_ADC_CALIB_ERASE_HI   0x2C		/* Objects 0x2C.. */
#define OD_ADC_CALIB_WR_ENA_HI  0x2D		/* Objects 0x2D.. */
#define OD_ADC_CALIB_WR_ENA_LO  0x00		/* Object  0x2D00 */

/* CRC */
#define OD_CALC_CRC_HI          0x30		/* Objects 0x30.. */
#define OD_CALC_CRC_LO          0x00		/* Object  0x3000 */
#define OD_CRC_MASTER_FLASH     1
#define OD_CRC_SLAVE_FLASH      2
#define OD_CRC_MASTER_FLASH_GET 3

/* ELMB Serial Number */
#define OD_ELMB_SERIAL_NO_HI    0x31		/* Objects 0x31.. */
#define OD_ELMB_SERIAL_NO_LO    0x00		/* Object  0x3100 */
#define OD_ELMB_SN_WRITE_ENA_LO 0x01		/* Object  0x3101 */

/* CAN-controller configuration */
#define OD_CAN_CONFIG_HI        0x32		/* Objects 0x32.. */
#define OD_CAN_CONFIG_LO        0x00		/* Object  0x3200 */

/* Other */
#define OD_COMPILE_OPTIONS_HI   0x5C		/* Objects 0x5C.. */
#define OD_COMPILE_OPTIONS_LO   0x00		/* Object  0x5C00 */

#define OD_TEST_HI              0x5D		/* Objects 0x5D.. */
#define OD_TEST_LO              0xFF		/* Object  0x5DFF */
#define OD_IO_TEST              1

#define OD_SWITCH_TO_LOADER_HI  0x5E		/* Objects 0x5E.. */
#define OD_SWITCH_TO_LOADER_LO  0x00		/* Object  0x5E00 */

#define OD_PROGRAM_CODE_HI      0x5F		/* Objects 0x5F.. */
#define OD_PROGRAM_CODE_LO      0x50		/* Object  0x5F50 */

/* ============================================================== */
/* Standardised CANopen Device Profile objects */

#define OD_DIGITAL_IN_HI        0x60		/* Objects 0x60.. */
#define OD_DIGITAL_IN_8_LO      0x00		/* Object  0x6000 */
#define OD_DIGIN_INTRPT_ENA_LO  0x05		/* Object  0x6005 */
#define OD_DIGIN_INTRPT_MSK_LO  0x06		/* Object  0x6006 */

#define OD_DIGITAL_OUT_HI       0x62		/* Objects 0x62.. */
#define OD_DIGITAL_OUT_8_LO     0x00		/* Object  0x6200 */
#define OD_DIGITAL_OUT_8_MSK_LO 0x08		/* Object  0x6208 */

#define OD_ANALOG_HI            0x64		/* Objects 0x64.. */
#define OD_ANALOG_IN_LO         0x04		/* Object  0x6404 */
#define OD_ANALOG_OUT_8_LO      0x10		/* Object  0x6410 */
#define OD_ANALOG_OUT_16_LO     0x11		/* Object  0x6411 */
#define OD_ANALOG_OUT_32_LO     0x12		/* Object  0x6412 */
#define OD_ANALOG_IN_INT_ENA_LO 0x23		/* Object  0x6423 */
#define OD_ANALOG_IN_UPPER_LO   0x24		/* Object  0x6424 */
#define OD_ANALOG_IN_LOWER_LO   0x25		/* Object  0x6425 */
#define OD_ANALOG_IN_DELTA_LO   0x26		/* Object  0x6426 */

/* ============================================================== */
/* Emergency objects: stuff for the Manufacturer-specific Error Field
   (first byte) */

#define EMG_SLAVE_PROCESSOR     0x20

#define EMG_CRC                 0x30

#define EMG_EEPROM_WRITE_PARS   0x41
#define EMG_EEPROM_READ_PARS    0x42

#define EMG_IRREGULAR_RESET     0xF0
#define EMG_NO_BOOTLOADER       0xF1

#endif /* OBJECTS_H */
/* ------------------------------------------------------------------------ */
