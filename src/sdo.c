/* ------------------------------------------------------------------------
File   : sdo.c

Descr  : The CANopen SDO server, which serves read/write requests to the
	 Object Dictionary.

History: 25JAN.00; Henk B&B; Start of development of a version for the ELMB.
--------------------------------------------------------------------------- */

#include "general.h"
#include "adc_cal.h"
#include "app.h"
#include "can.h"
#include "crc.h"
#include "guarding.h"
#include "objects.h"
#include "pdo.h"
#include "serialno.h"
#include "store.h"
#include "timer1XX.h"
#include "watchdog.h"

#ifdef _2313_SLAVE_PRESENT_
#include "download.h"
#endif /* _2313_SLAVE_PRESENT_ */

#ifdef _INCLUDE_TESTS_
#include "iotest.h"
#endif

/* Parameters for Segmented SDO transfer */
static UINT16 NbytesSeg = (UINT16) 0; /* Number of bytes to be transferred */
static BYTE   OdIndexHiSeg, OdIndexLoSeg, OdSubindSeg; /* Object in transfer */
static BYTE   ToggleSeg; /* Toggle bit for Segmented-SDO */
static BOOL   FirstSeg;  /* May be important for application to know */
static BOOL   UploadSeg; /* Remember if currently Up- or Downloading */

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BYTE sdo_read( BYTE *msg_data );
static BYTE sdo_write( BYTE *msg_data, BYTE *error_class );
static BYTE sdo_expedited_write( BYTE *msg_data );
static BYTE sdo_segmented_init( BYTE *msg_data );
static BYTE sdo_segmented_read( BYTE *msg_data, BYTE *error_class );
static BYTE sdo_segmented_write( BYTE *msg_data, BYTE *error_class );
static void sdo_abort( BYTE error_class,
		       BYTE error_code,
		       BYTE *msg_data );

static void jump_to_bootloader( void );

/* ------------------------------------------------------------------------ */

void sdo_server( BYTE *msg_data )
{
  BYTE sdo_mode, sdo_error, sdo_eclass, cs;

  /* Preset error class identifier */
  sdo_eclass = SDO_ECLASS_ACCESS;

  /* SDO modifier bits are in the first data byte */
  sdo_mode = msg_data[0];

  /* Determine the command specifier (cs) from
     the SDO modifier bits in the first byte */
  cs = sdo_mode & SDO_COMMAND_SPECIFIER_MASK;

  switch( cs )
    {
    case SDO_INITIATE_UPLOAD_REQ:
      /* ==> Read from the Object Dictionary <== */

      /* Reset any ongoing Segmented SDO */
      NbytesSeg = (UINT16) 0;

      /* Both Expedited transfer (data: 4 bytes or less) or Segmented:
	 the local app software on this node decides what's it going to be */
      sdo_error = sdo_read( msg_data );

      break;

    case SDO_INITIATE_DOWNLOAD_REQ:
      /* ==> Write to the Object Dictionary <== */

      /* Reset any ongoing Segmented SDO */
      NbytesSeg = (UINT16) 0;

      /* Both Expedited transfer (data: 4 bytes or less) or Segmented */
      sdo_error = sdo_write( msg_data, &sdo_eclass );

      break;

    case SDO_DOWNLOAD_SEGMENT_REQ:
      /* ==> Write to the Object Dictionary (segmented) <== */
      if( (UploadSeg & TRUE) == FALSE )
	{
	  sdo_error = sdo_segmented_write( msg_data, &sdo_eclass );

	  /* Reset the ongoing Segmented SDO if necessary
	     and fill in object (sub)index for Abort Transfer message */
	  if( sdo_error != SDO_ECODE_OKAY )
	    {
	      NbytesSeg = (UINT16)0;
	      msg_data[1] = OdIndexLoSeg;
	      msg_data[2] = OdIndexHiSeg;
	      msg_data[3] = OdSubindSeg;
	    }
	}
      else
	{
	  /* Download while we're uploading ? I don't think so... */
	  sdo_error  = SDO_ECODE_ACCESS;
	  sdo_eclass = SDO_ECLASS_SERVICE;
	}
      break;

    case SDO_UPLOAD_SEGMENT_REQ:
      /* ==> Read from the Object Dictionary (segmented) <== */
      if( (UploadSeg & TRUE) == TRUE )
	{
	  sdo_error = sdo_segmented_read( msg_data, &sdo_eclass );

	  /* Reset the ongoing Segmented SDO if necessary
	     and fill in object (sub)index for Abort Transfer message */
	  if( sdo_error != SDO_ECODE_OKAY )
	    {
	      NbytesSeg = (UINT16)0;
	      msg_data[1] = OdIndexLoSeg;
	      msg_data[2] = OdIndexHiSeg;
	      msg_data[3] = OdSubindSeg;
	    }
	}
      else
	{
	  /* Upload while we're downloading ? I don't think so... */
	  sdo_error  = SDO_ECODE_ACCESS;
	  sdo_eclass = SDO_ECLASS_SERVICE;
	}
      break;

    case SDO_ABORT_TRANSFER:
      /* Reset any ongoing Segmented SDO */
      NbytesSeg = (UINT16) 0;

      return; /* Unconfirmed service */

    default:
      /* Reset any ongoing Segmented SDO */
      NbytesSeg = (UINT16) 0;

      /* Unknown command specifier !? */
      sdo_error  = SDO_ECODE_PAR_ILLEGAL;
      sdo_eclass = SDO_ECLASS_SERVICE;
      break;
    }

  /* Send the SDO reply... */
  if( sdo_error == SDO_ECODE_OKAY )
    can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data ); /* All went okay */
  else
    sdo_abort( sdo_eclass, sdo_error, msg_data );    /* Aborted... */
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_read( BYTE *msg_data )
{
  BYTE sdo_error, nbytes;
  BYTE od_index_hi, od_index_lo, od_subind;
  BOOL segmented = FALSE;

  /* No error */
  sdo_error   = SDO_ECODE_OKAY;

  /* Extract Object Dictionary indices */
  od_index_lo = msg_data[1];
  od_index_hi = msg_data[2];
  od_subind   = msg_data[3];

  /* Initialise data bytes to zero */
  msg_data[4] = 0;
  msg_data[5] = 0;
  msg_data[6] = 0;
  msg_data[7] = 0;

  /* Default number of significant bytes:
     set to a different value if it saves more statements,
     now default assuming 32-bit data item... */
  nbytes = 4;

  /* Get the requested object */
  switch( od_index_hi )
    {
    case OD_DEVICE_INFO_HI:
      switch( od_index_lo )
	{
	case OD_DEVICE_TYPE_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = DEVICE_TYPE_CHAR0;
	      msg_data[5] = DEVICE_TYPE_CHAR1;
	      msg_data[6] = DEVICE_TYPE_CHAR2;
	      msg_data[7] = DEVICE_TYPE_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ERROR_REG_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = CANopenErrorReg;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STATUS_REG_LO:
	  if( od_subind == 0 )
	    {
	      app_status( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DEVICE_NAME_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_DEV_NAME_CHAR0;
	      msg_data[5] = MNFCT_DEV_NAME_CHAR1;
	      msg_data[6] = MNFCT_DEV_NAME_CHAR2;
	      msg_data[7] = MNFCT_DEV_NAME_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_HW_VERSION_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_HARDW_VERSION_CHAR0;
	      msg_data[5] = MNFCT_HARDW_VERSION_CHAR1;
	      msg_data[6] = MNFCT_HARDW_VERSION_CHAR2;
	      msg_data[7] = MNFCT_HARDW_VERSION_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_SW_VERSION_LO:
	  if( od_subind == 0 )
	    {
	      msg_data[4] = MNFCT_SOFTW_VERSION_CHAR0;
	      msg_data[5] = MNFCT_SOFTW_VERSION_CHAR1;
	      msg_data[6] = MNFCT_SOFTW_VERSION_CHAR2;
	      msg_data[7] = MNFCT_SOFTW_VERSION_CHAR3;
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_GUARDTIME_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_guardtime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_LIFETIME_FACTOR_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_lifetime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STORE_PARAMETERS_LO:
	case OD_DFLT_PARAMETERS_LO:
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 3;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	    case 2:
	    case 3:
	      /* Device saves parameters on command (OD_STORE_PARAMETERS),
		 restores parameters (OD_DFLT_PARAMETERS) */
	      msg_data[4] = 0x01;

	      /* ###??? Device saves parameters autonomously
		 (OD_STORE_PARAMETERS_LO) */
	      /*if( od_ind_lo == OD_STORE_PARAMETERS_LO )
		msg_data[4] = 0x03; */
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	case OD_HEARTBEAT_TIME_LO:
	  if( od_subind == 0 )
	    {
	      nbytes = guarding_get_heartbeattime( &msg_data[4] );
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_IDENTITY_LO:
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 1;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	      msg_data[4] = 0x78;
	      msg_data[5] = 0x56;
	      msg_data[6] = 0x34;
	      msg_data[7] = 0x12;
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_RPDO_PAR_HI:
      if( od_index_lo < RPDO_CNT )
	{
	  if( rpdo_get_comm_par( od_index_lo, od_subind,
				  &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_RPDO_MAP_HI:
      if( od_index_lo < RPDO_CNT )
	{
	  if( rpdo_get_mapping( od_index_lo, od_subind,
				 &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_TPDO_PAR_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_get_comm_par( od_index_lo, od_subind,
				  &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_TPDO_MAP_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_get_mapping( od_index_lo, od_subind,
				 &nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_CAN_CONFIG_HI:
      if( od_index_lo == OD_CAN_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 3;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 1:
	      msg_data[4] = can_get_rtr_disabled();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 2:
	      msg_data[4] = can_get_opstate_init();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    case 3:
	      msg_data[4] = can_get_busoff_maxcnt();
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_CALC_CRC_HI:
      if( od_index_lo == OD_CALC_CRC_LO )
	{
	  UINT16 crc;
	  BYTE   result;

	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      msg_data[4] = 2;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;

	    case OD_CRC_MASTER_FLASH:
	    case OD_CRC_SLAVE_FLASH:
	      if( od_subind == OD_CRC_MASTER_FLASH )
		result = crc_master( &crc );
	      else
		result = crc_slave( &crc );

	      if( result == FALSE )
		{
		  /* Something went wrong... */
		  if( crc == (UINT16) 0 )
		    {
		      /* No CRC found... */
		      sdo_error = SDO_ECODE_ACCESS;
		    }
		  else
		    {
		      /* Access error while reading Master FLASH */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  nbytes = 2;  /* Significant bytes < 4 */
		  msg_data[4] = (BYTE) (crc & (UINT16) 0x00FF);
		  msg_data[5] = (BYTE) ((crc & (UINT16) 0xFF00) >> 8);
		}
	      break;

	    case OD_CRC_MASTER_FLASH_GET:
	      result = crc_get( &msg_data[4] );
	      if( result == FALSE )
		{
		  /* No CRC found... */
		  sdo_error = SDO_ECODE_ACCESS;
		}
	      nbytes = 2;  /* Significant bytes < 4 */
	      break;

	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ELMB_SERIAL_NO_HI:
      if( od_index_lo == OD_ELMB_SERIAL_NO_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( sn_get_serial_number( &msg_data[4] ) == FALSE )
		{
		  /* EEPROM read operation failed
		     or Serial Number simply not present */
		  sdo_error = SDO_ECODE_HARDWARE;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_PARS_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  if( od_subind == OD_NO_OF_ENTRIES )
	    {
	      msg_data[4] = 4;
	      nbytes = 1;  /* Significant bytes < 4 */
	    }
	  else
	    {
	      if( od_subind <= STORE_ADC_CALIB_PARS )
		{
		  if( adc_get_calib_const( od_index_lo, od_subind-1,
					   &msg_data[4], TRUE ) == FALSE )
		    {
		      /* EEPROM read operation failed
			 or calibration constant simply not present */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* The sub-index does not exist */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_COMPILE_OPTIONS_HI:
      if( od_index_lo == OD_COMPILE_OPTIONS_LO )
	{
	  if( od_subind == 0 )
	    {
#ifdef _7BIT_NODEID_
	      msg_data[4] |= 0x20;
#endif
#ifdef _ELMB103_
	      msg_data[4] |= 0x80;
#endif
#ifdef _VARS_IN_EEPROM_
	      msg_data[5] |= 0x01;
#endif
#ifdef _INCLUDE_TESTS_
	      msg_data[5] |= 0x04;
#endif
#ifdef _CAN_REFRESH_
	      msg_data[5] |= 0x10;
#endif
#ifdef _2313_SLAVE_PRESENT_
	      msg_data[5] |= 0x20;
#endif
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

#ifdef _INCLUDE_TESTS_
    case OD_TEST_HI:
      /* Some (self)tests can be performed on I/O and memory, etc... */ 

      if( od_index_lo == OD_TEST_LO )
	{
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      /* The number of tests available */
	      msg_data[4] = 1;
	      nbytes = 1;  /* Significant bytes < 4 */
	      break;

	    case OD_IO_TEST:
	      /* Do a predefined test on all available I/O PORTs and PINs;
		 this can be one of the production acceptance tests
		 of the (ELMB +) Motherboard */
	      iotest( &msg_data[4] );
	      break;

	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;
#endif /* _INCLUDE_TESTS_ */

    default:
      sdo_error = app_sdo_read( od_index_hi, od_index_lo, od_subind,
				&msg_data[4], &nbytes, &segmented );
      if( sdo_error == SDO_ECODE_OKAY && segmented && nbytes == 4 )
	{
	  /* This will be a Segmented SDO upload: initialize its parameters */
	  sdo_error = sdo_segmented_init( msg_data );
	  UploadSeg = TRUE; /* Uploading... */
	}
      break;
    }

  /* Set appropriate SDO command specifier for reply... */
  msg_data[0] = SDO_INITIATE_UPLOAD_RESP;
  if( segmented == FALSE ) msg_data[0] |= SDO_EXPEDITED;

  /* ...and segment size (count of non-significant bytes) */
  msg_data[0] |= (SDO_SEGMENT_SIZE_INDICATED |
		  ((4-nbytes) << SDO_DATA_SIZE_SHIFT));

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_write( BYTE *msg_data, BYTE *error_class )
{
  BYTE sdo_error, sdo_mode, nbytes;

  sdo_error = SDO_ECODE_OKAY;
  sdo_mode  = msg_data[0];

  if( sdo_mode & SDO_EXPEDITED )
    {
      /* Expedited transfer (data: 4 bytes or less) */
      sdo_error = sdo_expedited_write( msg_data );
    }
  else
    {
      if( sdo_mode & SDO_SEGMENT_SIZE_INDICATED )
	{
	  /* Start of segmented transfer (OD write) */
	  sdo_error = sdo_segmented_init( msg_data );
	  if( sdo_error != SDO_ECODE_OKAY )
	    {
	      *error_class = SDO_ECLASS_SERVICE;
	    }
	  else
	    {
	      /* Determine if this is an object that can handle
		 a Segmented SDO Download of this length */
	      sdo_error = app_sdo_write_seg_init( OdIndexHiSeg, OdIndexLoSeg,
						  OdSubindSeg, NbytesSeg );
	      UploadSeg = FALSE; /* Downloading... */
	    }

	  /* Set appropriate SDO command specifier for reply */
	  msg_data[0] = SDO_INITIATE_DOWNLOAD_RESP;

	  /* CANopen: bytes 4 to 7 reserved, so set to zero */
	  for( nbytes=4; nbytes<8; ++nbytes ) msg_data[nbytes] = 0;
	}
      else
	{
	  /* s=0: reserved by CiA */
	  *error_class = SDO_ECLASS_SERVICE;
	  sdo_error    = SDO_ECODE_PAR_INCONSISTENT;
	}
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_expedited_write( BYTE *msg_data )
{
  BYTE sdo_error, sdo_mode, nbytes;
  BYTE od_index_hi, od_index_lo, od_subind;

  /* No error */
  sdo_error = SDO_ECODE_OKAY;

  /* Get the number of significant bytes */
  sdo_mode = msg_data[0];
  if( sdo_mode & SDO_DATA_SIZE_INDICATED )
    /* Size indicated */
    nbytes = 4-((sdo_mode & SDO_DATA_SIZE_MASK)>>SDO_DATA_SIZE_SHIFT);
  else
    /* If number of bytes is zero, size was not indicated... */
    nbytes = 0;

  /* Extract Object Dictionary indices */
  od_index_lo = msg_data[1];
  od_index_hi = msg_data[2];
  od_subind   = msg_data[3];

  /* Write the requested object */
  switch( od_index_hi )
    {
    case OD_TPDO_PAR_HI:
      if( od_index_lo < TPDO_CNT )
	{
	  if( tpdo_set_comm_par( od_index_lo, od_subind,
				 nbytes, &msg_data[4] ) == FALSE )
	    {
	      /* The subindex does not exist or the number of bytes
		 is incorrect or the parameter could not be written */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

#ifdef _2313_SLAVE_PRESENT_
    case OD_PROGRAM_CODE_HI:
      if( od_index_lo == OD_PROGRAM_CODE_LO )
	{
	  if( od_subind == 1 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  if( do_serial_instruction( &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;
#endif /* _2313_SLAVE_PRESENT_ */

    case OD_SWITCH_TO_LOADER_HI:
      if( od_index_lo == OD_SWITCH_TO_LOADER_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
#ifdef _2313_SLAVE_PRESENT_
		  /* Disable Timer1 interrupt to stop
		     the Slave aliveness-check mechanism:
		     Slave should take control of the node,
		     after some time, unless.... */
		  timer1_stop();
#endif /* _2313_SLAVE_PRESENT_ */

		  /* Send a reply before making the jump... */
		  msg_data[0] = SDO_INITIATE_DOWNLOAD_RESP;
		  msg_data[4] = 0;
		  can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
		  timer2_delay_ms( 5 );

		  /* There is a Bootloader: it will take control
		     (and also keep the Slave happy, if present) */
		  jump_to_bootloader();
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_DEVICE_INFO_HI:
      switch( od_index_lo )
	{
	case OD_LIFETIME_FACTOR_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  /* Set new Life Time Factor */
		  if( guarding_set_lifetime( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_STORE_PARAMETERS_LO:
	  if( od_subind == 1 || od_subind == 2 || od_subind == 3 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Check for correct signature */
		  if( msg_data[4] == 's' && msg_data[5] == 'a' &&
		      msg_data[6] == 'v' && msg_data[7] == 'e' )
		    {
		      if( storage_save_parameters( od_subind ) == FALSE )
			{
			  /* Something went wrong */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		    }
		  else
		    {
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_DFLT_PARAMETERS_LO:
	  if( od_subind == 1 || od_subind == 2 || od_subind == 3 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Check for correct signature */
		  if( msg_data[4] == 'l' && msg_data[5] == 'o' &&
		      msg_data[6] == 'a' && msg_data[7] == 'd' )
		    {
		      if( storage_set_defaults( od_subind ) == FALSE )
			{
			  /* Something went wrong */
			  sdo_error = SDO_ECODE_HARDWARE;
			}
		    }
		  else
		    {
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_HEARTBEAT_TIME_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes == 2 || nbytes == 0 )
		{
		  /* Set new Heartbeat Time */
		  if( guarding_set_heartbeattime( &msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_CAN_CONFIG_HI:
      if( od_index_lo == OD_CAN_CONFIG_LO )
	{
	  switch( od_subind )
	    {
	    case 1:
	      if( nbytes <= 1 )
		{
		  if( can_set_rtr_disabled( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_TYPE_CONFLICT;
	      break;
	    case 2:
	      if( nbytes <= 1 )
		{
		  if( can_set_opstate_init( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_TYPE_CONFLICT;
	      break;
	    case 3:
	      if( nbytes <= 1 )
		{
		  if( can_set_busoff_maxcnt( msg_data[4] ) == FALSE )
		    sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      else
		/* Wrong number of bytes provided */
		sdo_error = SDO_ECODE_TYPE_CONFLICT;
	      break;
	    default:
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	      break;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ELMB_SERIAL_NO_HI:
      switch( od_index_lo )
	{
	case OD_ELMB_SERIAL_NO_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  /* Set the ELMB Serial Number */
		  if( sn_set_serial_number( &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	case OD_ELMB_SN_WRITE_ENA_LO:
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  /* Enable a write-operation to the ELMB Serial Number */
		  if( sn_serial_number_write_enable( msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	  break;

	default:
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	  break;
	}
      break;

    case OD_ADC_CALIB_PARS_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  /* Check for valid parameters */
	  if( od_subind != 0 && od_subind <= STORE_ADC_CALIB_PARS )
	    {
	      if( nbytes == 4 || nbytes == 0 )
		{
		  if( adc_set_calib_const( od_index_lo, od_subind-1,
					   &msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong while writing to EEPROM */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_ERASE_HI:
      if( od_index_lo < STORE_ADC_CALIB_BLOCKS )
	{
	  /* Check for valid parameters */
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_erase_calib_const( od_index_lo,
					     msg_data[4] ) == FALSE )
		    {
		      /* Something went wrong while writing to EEPROM */
		      sdo_error = SDO_ECODE_HARDWARE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    case OD_ADC_CALIB_WR_ENA_HI:
      if( od_index_lo == OD_ADC_CALIB_WR_ENA_LO )
	{
	  if( od_subind == 0 )
	    {
	      if( nbytes <= 1 )
		{
		  if( adc_calib_const_write_enable( msg_data[4] ) == FALSE )
		    {
		      /* Something wrong with parameters */
		      sdo_error = SDO_ECODE_ATTRIBUTE;
		    }
		}
	      else
		{
		  /* Wrong number of bytes provided */
		  sdo_error = SDO_ECODE_TYPE_CONFLICT;
		}
	    }
	  else
	    {
	      /* The sub-index does not exist */
	      sdo_error = SDO_ECODE_ATTRIBUTE;
	    }
	}
      else
	{
	  /* The index can not be accessed, does not exist */
	  sdo_error = SDO_ECODE_NONEXISTENT;
	}
      break;

    default:
      sdo_error = app_sdo_write_exp( od_index_hi, od_index_lo, od_subind,
				     &msg_data[4], nbytes );
      break;
    }

  /* Set appropriate SDO command specifier for reply */
  msg_data[0] = SDO_INITIATE_DOWNLOAD_RESP;

  /* CANopen: bytes 4 to 7 reserved, so set to zero, except when programming
     the Slave: the SDO reply possibly contains a read memory byte... */
  if( od_index_hi != OD_PROGRAM_CODE_HI )
    {
      msg_data[4] = 0;
      msg_data[5] = 0;
      msg_data[6] = 0;
      msg_data[7] = 0;
    }

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_segmented_init( BYTE *msg_data )
{
  /* Set local Segmented-SDO parameters for a subsequent
     Segmented read or write */
  BYTE sdo_error;

  /* Initialize */
  FirstSeg  = TRUE;
  ToggleSeg = SDO_TOGGLE_BIT;
  sdo_error = SDO_ECODE_OKAY;

  /* Extract Object Dictionary indices */
  OdIndexLoSeg = msg_data[1];
  OdIndexHiSeg = msg_data[2];
  OdSubindSeg  = msg_data[3];

  /* Extract byte counter: number of bytes to be expected;
     in this app we will not handle more than 65535 bytes */
  if( msg_data[6] != 0 || msg_data[7] != 0 ) sdo_error = SDO_ECODE_PAR_ILLEGAL;
  NbytesSeg = ((UINT16) msg_data[4]) + ((UINT16) msg_data[5] << 8);

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_segmented_read( BYTE *msg_data, BYTE *error_class )
{
  BYTE sdo_error, sdo_mode, nbytes, last_segment;

  sdo_error = SDO_ECODE_OKAY;
  sdo_mode  = msg_data[0];

  /* Toggle bit: toggle it and check against the received toggle bit.. */
  ToggleSeg ^= SDO_TOGGLE_BIT;
  if( (sdo_mode & SDO_TOGGLE_BIT) != (ToggleSeg & SDO_TOGGLE_BIT) ) 
    {
      /* Error in toggle bit */
      *error_class = SDO_ECLASS_SERVICE;
      return SDO_ECODE_PAR_INCONSISTENT;
    }

  /* Check the byte counter */
  if( NbytesSeg == (UINT16) 0 )
    {
      /* No more bytes to deliver */
      *error_class = SDO_ECLASS_SERVICE;
      return SDO_ECODE_PAR_ILLEGAL;
    }

  /* Initialise data bytes to zero */
  for( nbytes=1; nbytes<8; ++nbytes ) msg_data[nbytes] = 0;

  /* Read the requested object (segmented) */
  sdo_error = app_sdo_read_seg( OdIndexHiSeg, OdIndexLoSeg, OdSubindSeg,
				&msg_data[1], &nbytes, FirstSeg );

  if( sdo_error == SDO_ECODE_OKAY )
    {
      FirstSeg = FALSE;

      /* Check and update the byte counter */
      if( nbytes > 7 || NbytesSeg < (UINT16) nbytes )
	{
	  /* Something wrong in number of bytes returned from the app code */
	  sdo_error = SDO_ECODE_TYPE_CONFLICT;
	  NbytesSeg = (UINT16) 0;
	}
      else
	{
	  NbytesSeg -= (UINT16) nbytes;
	}

      /* Last segment? */
      if( NbytesSeg == (UINT16) 0 )
	last_segment = SDO_LAST_SEGMENT;
      else
	last_segment = 0;

      /* Set appropriate SDO command specifier for reply... */
      msg_data[0] = (SDO_UPLOAD_SEGMENT_RESP | (ToggleSeg & SDO_TOGGLE_BIT) |
		     last_segment);

      /* ...and segment size (count of non-significant bytes) */
      msg_data[0] |= ((7-nbytes) << SDO_SEGMENT_SIZE_SHIFT);
    }

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BYTE sdo_segmented_write( BYTE *msg_data, BYTE *error_class )
{
  BYTE sdo_error, sdo_mode, nbytes;

  sdo_error = SDO_ECODE_OKAY;
  sdo_mode  = msg_data[0];

  /* Toggle bit: toggle it and check against the received toggle bit.. */
  ToggleSeg ^= SDO_TOGGLE_BIT;
  if( (sdo_mode & SDO_TOGGLE_BIT) != (ToggleSeg & SDO_TOGGLE_BIT) ) 
    {
      /* Error in toggle bit */
      *error_class = SDO_ECLASS_SERVICE;
      return SDO_ECODE_PAR_INCONSISTENT;
    }

  /* The number of (non-)significant bytes in this segment */
  nbytes = ((sdo_mode & SDO_SEGMENT_SIZE_MASK)>>SDO_SEGMENT_SIZE_SHIFT);
  if( nbytes == 0 )
    {
      /* No size indicated: set to maximum or to whatever still expected */
      if( NbytesSeg < (UINT16) nbytes )
	nbytes = (BYTE) NbytesSeg;
      else
	nbytes = 7;
    }
  else
    {
      nbytes = 7-nbytes;
    }

  /* Check the byte counter */
  if( NbytesSeg < (UINT16) nbytes || NbytesSeg == (UINT16) 0 )
    {
      /* More bytes than we expected */
      *error_class = SDO_ECLASS_SERVICE;
      return SDO_ECODE_PAR_ILLEGAL;
    }

  /* Check for last segment and update the byte counter */
  if( sdo_mode & SDO_LAST_SEGMENT )
    NbytesSeg = (UINT16) 0; /* Don't accept anymore segments */
  else
    NbytesSeg -= (UINT16) nbytes;

  /* Write the requested object (segmented) */
  sdo_error = app_sdo_write_seg( OdIndexHiSeg, OdIndexLoSeg, OdSubindSeg,
				 &msg_data[1], nbytes, FirstSeg );

  if( sdo_error == SDO_ECODE_OKAY )
    {
      FirstSeg = FALSE;

      /* Set appropriate SDO command specifier for reply */
      msg_data[0] = SDO_DOWNLOAD_SEGMENT_RESP | (ToggleSeg & SDO_TOGGLE_BIT);

      /* CANopen: bytes 1 to 7 reserved, so set to zero */
      for( nbytes=1; nbytes<8; ++nbytes ) msg_data[nbytes] = 0;
    }

  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static void sdo_abort( BYTE error_class,
		       BYTE error_code,
		       BYTE *msg_data )
{
  msg_data[0] = SDO_ABORT_TRANSFER;

  /* msg_data[1], msg_data[2], msg_data[3] should contain
     index and sub-index: leave intact */

  /* Error class */
  msg_data[7] = error_class;

  /* Error code */
  msg_data[6] = error_code;

  /* Additional code, not filled in for the time being */
  msg_data[5] = 0;
  msg_data[4] = 0;

  can_write( C91_SDOTX, C91_SDOTX_LEN, msg_data );
}

/* ------------------------------------------------------------------------ */
/* Function call which results in a jump to address 0xF000
   which starts the Bootloader program
   (provided the Bootloader size is set (by the fuses) to 4 kWords!) */

static void jump_to_bootloader( void )
{
  /* This does not apply to an ELMB with ATmega103 microcontroller */
#ifndef _ELMB103_
  BYTE flashbyte;

  /* Set (byte) address in the proper registers (for ELPM access) */
  asm( "ldi R30,0x00" );
  asm( "ldi R31,0xE0" );

  /* Set RAMPZ register to access the upper 64k page of program memory */
  RAMPZ = 1;

  /* Read the program memory byte and store it in 'flashbyte' */
  asm( "elpm" );
  asm( "mov %flashbyte, R0" );

  /* Reset RAMPZ register */
  RAMPZ = 0;

  /* If there is no Bootloader, return to the user application ! */
  if( flashbyte == 0xFF )
    {
      /* CANopen Error Code 0x6000: device software */
      can_write_emergency( 0x00, 0x50, EMG_NO_BOOTLOADER,
			   0, 0, 0, ERRREG_MANUFACTURER );
      return;
    }
  
  /* Disable watchdog timer (if possible) */
  watchdog_disable();

  /* Disable all interrupts */
  CLI();

  /* Z-pointer: 0xF000 (word address) */
  asm( "ldi R30,0x00" );
  asm( "ldi R31,0xF0" );
  
  /* Jump to the Bootloader at (word) address 0xF000 */
  asm( "ijmp" );
#endif /* _ELMB103_ */
}

/* ------------------------------------------------------------------------ */
