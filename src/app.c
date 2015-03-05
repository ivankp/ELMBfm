/* ------------------------------------------------------------------------
File   : app.c

Descr  : User application functions.
         - They need filling in.
	 - Functions to add may include for example more functions called
	   from the SDO server to read and write data and parameters from
	   and to your hardware; examples below are app_get() and app_set().
	 - The basic low-level access functions for your hardware are
	   in a separate file and included by include file
	   "your_lowlevel_hardware_functions.h", as shown below.
	 - In this example code TPDO1 is used for multi-channel readout
	   operations (multiple PDO messages are generated when the appropriate
	   trigger occurs), but any or none of the TPDOs could be used
	   in this fashion.
	 - Functions 'app_sdo_read()', 'app_sdo_read_seg()',
	   'app_sdo_write_exp()', 'app_sdo_write_seg_init()' and
	   'app_sdo_write_seg()' are the SDO server functions for
	   read/write access to Object Dictionary items concerning
	   the user application part;
	   'Expedited Transfer' (data items up to a size of 4 bytes)
	   as well as 'Segmented Transfer' is supported.


History: ..JAN.03; username; Definition.
--------------------------------------------------------------------------- */

#include "general.h"
#include "app.h"
#include "can.h"
#include "eeprom.h"
#include "objects.h"
#include "pdo.h"
#include "store.h"

/* Include the functions that access your (custom) hardware */
//#include "your_lowlevel_hardware_functions.h"

/* ------------------------------------------------------------------------ */
/* Globals */

/* Per PDO the number of mapped objects */
/* ...fill in.... */
const BYTE   PDOMAP_CNT[TPDO_CNT+RPDO_CNT] = { 2, 2, 2, 2,
					       2, 2, 2, 2 };

/* Per PDO the mapped objects */
/* ...fill in.... */
const UINT32 PDOMAP[TPDO_CNT+RPDO_CNT][APP_MAX_MAPPED_CNT] =
{
  { 0x60000108L, 0x60000208L }, /* example: Digital Inputs: 1-8, 9-16 */
  { 0x60000108L, 0x60000208L }, /* example: Digital Inputs: 1-8, 9-16 */
  { 0x60000108L, 0x60000208L }, /* example: Digital Inputs: 1-8, 9-16 */
  { 0x60000108L, 0x60000208L }, /* example: Digital Inputs: 1-8, 9-16 */
  { 0x62000108L, 0x62000208L }, /* example: Digital Outputs: 1-8, 9-16 */
  { 0x62000108L, 0x62000208L }, /* example: Digital Outputs: 1-8, 9-16 */
  { 0x62000108L, 0x62000208L }, /* example: Digital Outputs: 1-8, 9-16 */
  { 0x62000108L, 0x62000208L }  /* example: Digital Outputs: 1-8, 9-16 */
};

/* Application parameter example: total number of channels */
static BYTE AppChans;     /* (copy in EEPROM) */

/* Storage space for error bits concerning the hardware */
static BYTE AppError;

/* ------------------------------------------------------------------------ */
/* Global variables for multi-channel readout operations */

/* Channel index */
static BYTE AppChanNo;

/* Scanning-operation-in-progress boolean */
static BOOL AppScanInProgress;

/* ------------------------------------------------------------------------ */
/* Global variables for array read/write operations by Segmented PDO */

/* Array size */
static UINT16 AppArrSz;

/* Array index */
static UINT16 AppArrIndex;

/* The data byte array */
static BYTE   AppArr[APP_ARR_SZ_MAX];

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static BOOL app_scan_next  ( void );
static BOOL app_get_par    ( BYTE index, BYTE *data, BYTE *no_of_bytes );
static BOOL app_set_par    ( BYTE index, BYTE *data );
static void app_load_config( void );

/* ------------------------------------------------------------------------ */

void app_init( void )
{
  AppError = 0;

  /* Initialize processor I/O pins involved in interfacing to the hardware */
  /* ...fill in.... */

  /* Initialise configuration parameters */
  app_load_config();

  /* Initialize variables for multi-channel readout operations */
  AppChanNo         = 0;
  AppScanInProgress = FALSE;

  /* Initialize array variables */
  AppArrSz = (UINT16) 0;

  /* Initialize your hardware */
  /* ...fill in.... */
}

/* ------------------------------------------------------------------------ */

BYTE app_status( BYTE *status )
{
  /* Use the bits in AppError (add more bytes if required)
     to report problems with your hardware */
  *status = AppError;
  return 1; /* Return number of bytes */
}

/* ------------------------------------------------------------------------ */

void app_rpdo1( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO received containing 'dlc' databytes in 'can_data[]':
     - write data from 'can_data[]' to your hardware
     - no reply message required */

  /* ...fill in.... */
}

/* ------------------------------------------------------------------------ */

void app_rpdo2( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO received containing 'dlc' databytes in 'can_data[]':
     - write data from 'can_data[]' to your hardware
     - no reply message required */

  /* ...fill in.... */
}

/* ------------------------------------------------------------------------ */

void app_rpdo3( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO received containing 'dlc' databytes in 'can_data[]':
     - write data from 'can_data[]' to your hardware
     - no reply message required */

  /* ...fill in.... */
}

/* ------------------------------------------------------------------------ */

void app_rpdo4( BYTE dlc, BYTE *can_data )
{
  /* Receive-PDO received containing 'dlc' databytes in 'can_data[]':
     - write data from 'can_data[]' to your hardware
     - no reply message required */

  /* ...fill in.... */
}

/* ------------------------------------------------------------------------ */

void app_tpdo1( void )
{
  /* This is an example of a Transmit-PDO that is used to read out
     multiple 'data channels':

     A call to app_tpdo_scan_start() will result in multiple calls to
     app_scan_next() during subsequent loops of the main function;
     global variable AppChanNo keeps track of which channel is to be read
     out next; global variable AppChans holds the number of channels
     to read out; if an ongoing 'scan' needs to be aborted
     function app_tpdo_scan_stop() is called.
     In case app_scan_next() only sends PDOs for data/channels that have
     changed or crossed certain limits, etc. the scanning-loop could
     be made to run continuously (only while the node is in Operational mode),
     sending messages to the host system only in case of such an event,
     releaving the host system from having to poll.

     Any of the other TPDOs could be used to do something similar;
     on the other hand, if only a single TPDO message is to be sent,
     use code as shown in app_tpdo2() in this function */

  /* Trigger a multi-channel readout for this PDO */
  app_tpdo_scan_start();
}

/* ------------------------------------------------------------------------ */

void app_tpdo2( void )
{
  BYTE pdo_data[C91_TPDO2_LEN];

  /* Read data from your hardware into pdo_data[] */
  /* ...fill in.... */
  {
    BYTE i;
    for( i=0; i<C91_TPDO2_LEN; ++i ) pdo_data[i] = i+1;
  }

  /* Send the Transmit-PDO */
  can_write( C91_TPDO2, C91_TPDO2_LEN, pdo_data );
}

/* ------------------------------------------------------------------------ */

void app_tpdo3( void )
{
  BYTE pdo_data[C91_TPDO3_LEN];

  /* Read data from your hardware into pdo_data[] */
  /* ...fill in.... */
  {
    BYTE i;
    for( i=0; i<C91_TPDO3_LEN; ++i ) pdo_data[i] = i+1;
  }

  /* Send the Transmit-PDO */
  can_write( C91_TPDO3, C91_TPDO3_LEN, pdo_data );
}

/* ------------------------------------------------------------------------ */

void app_tpdo4( void )
{
  BYTE pdo_data[C91_TPDO4_LEN];

  /* Read data from your hardware into pdo_data[] */
  /* ...fill in.... */
  {
    BYTE i;
    for( i=0; i<C91_TPDO4_LEN; ++i ) pdo_data[i] = i+1;
  }

  /* Send the Transmit-PDO */
  can_write( C91_TPDO4, C91_TPDO4_LEN, pdo_data );
}

/* ------------------------------------------------------------------------ */

void app_tpdo_on_cos( void )
{
  /* Send (a) PDO(s) in case of a 'change-of-state':
     any number of (different) PDOs could be generated here */

  BOOL change_of_state = FALSE;
  BYTE pdo_data[C91_TPDO2_LEN];

  /* Send a PDO on change-of-state of your hardware */
  /* ...fill in.... */

  if( change_of_state )
    {
      /* Send a Transmit-PDO */
      can_write( C91_TPDO2, C91_TPDO2_LEN, pdo_data );
    }
}

/* ------------------------------------------------------------------------ */

void app_tpdo_scan_start( void )
{
#ifdef _VARS_IN_EEPROM_
  /* Refresh variable with copy in EEPROM */
  AppChans = eeprom_read( EE_APP_CHANS );
#endif

  /* Start scanning only if scanning not already in progress
     and there are any channels to read out (this is probably configurable) */
  if( (AppScanInProgress & TRUE) == FALSE && AppChans > 0 )
    {
      /* Start a scan cycle */
      AppChanNo = 0;
      AppScanInProgress = app_scan_next();
    }
}

/* ------------------------------------------------------------------------ */

void app_tpdo_scan_stop( void )
{
  /* Abort a channel scan cycle in-progress:
     e.g. aborting ongoing ADC conversions, etc... */
  if( (AppScanInProgress & TRUE) == TRUE )
    {
      /* ...fill in.... */
    }

  /* Initialize variables for scanning operations */
  AppChanNo         = 0;
  AppScanInProgress = FALSE;
}

/* ------------------------------------------------------------------------ */

void app_tpdo_scan( void )
{
  /* Handle an on-going multi-channel scan */
  if( (AppScanInProgress & TRUE) == TRUE )
    {
      /* Read out next channel */
      AppScanInProgress = app_scan_next();
    }
}

/* ------------------------------------------------------------------------ */

static BOOL app_scan_next( void )
{
  BYTE pdo_data[C91_TPDO1_LEN];

  /* Send a TPDO with the next channel's data, if available */

  /* Postpone sending if necessary !
     (when previous message has not been sent yet) */
  if( can_transmitting(C91_TPDO1) ) return TRUE;

  /* Put the channel number in one of the PDO databytes */
  pdo_data[0] = AppChanNo;

  /* ...fill in.... */
  {
    BYTE i;
    for( i=1; i<C91_TPDO1_LEN; ++i ) pdo_data[i] = i+0x10;
  }

  /* Send a Transmit-PDO (TPDO1 chosen as an example) */
  can_write( C91_TPDO1, C91_TPDO1_LEN, pdo_data );

  ++AppChanNo;
  /* Are we done with the current scan cycle?,
     i.e. have all channels been read out ? */
  if( AppChanNo == AppChans )
    {
      /* Yes, we're done... */
      return FALSE;
    }
  else
    {
      /* No, not done yet... */
      return TRUE;
    }
}

/* ------------------------------------------------------------------------ */

BYTE app_sdo_read( BYTE od_index_hi,
		   BYTE od_index_lo,
		   BYTE od_subind,
		   BYTE data[4],
		   BYTE *nbytes,
		   BOOL *segmented )
{
  /* Data returned is stored in 'data[]' (up to 4 bytes);
     the number of significant bytes is returned as '*nbytes';
     the return value of the function is the SDO error code;
     'segmented' is set to TRUE if the app variable is to be read
     through a Segmented SDO; in that case 'data' contains the number
     of bytes to be read */

  BYTE sdo_error = SDO_ECODE_OKAY; /* No error */

  /* Default: Expedited Transfer */
  *segmented = FALSE;

  /* Default number of significant bytes:
     set to a different value if it saves more statements,
     now default assuming 32-bit data item... */
  *nbytes = 4;

  /* Read the requested object */
  switch( od_index_hi )
    {
    case OD_APP_HI:
      switch( od_index_lo )
	{
	case OD_APP_LO:
	  switch( od_subind )
	    {
	    case OD_NO_OF_ENTRIES:
	      data[0] = 1;
	      *nbytes = 1;  /* Significant bytes != 4 */
	      break;
	    case 1:
	    case 2:
	      /* Read an application parameter or data item */
	      if( app_get_par( od_subind, data, nbytes ) == FALSE )
		{
		  /* Something went wrong */
		  sdo_error = SDO_ECODE_HARDWARE;
		}
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

    case OD_APP_ARR_HI:
      switch( od_index_lo )
	{
	case OD_APP_ARR_LO:
	  switch( od_subind )
	    {
	    case 0:
	    case 1:
	      /* Read the example byte array of arbitrary length */
	      if( AppArrSz > (UINT16) 0 )
		{
#ifdef OPCSERVER_HANDLES_SEG_AND_EXP
		  if( AppArrSz <= (UINT16) 4 )
		    {
		      /* Expedited SDO */
		      BYTE i;

		      /* Reset array index */
		      AppArrIndex = (UINT16) 0;

		      /* Copy the array to the message data bytes,
			 and add 'od_subind' to each byte, just for fun... */
		      *nbytes = (BYTE) (AppArrSz & 0x00FF);
		      for( i=0; i<*nbytes; ++i, ++AppArrIndex )
			data[i] = AppArr[AppArrIndex] + od_subind;
		    }
		  else
#endif /* OPCSERVER_HANDLES_SEG_AND_EXP */
		    {
		      /* To be read by Segmented SDO: return expected size
			 in bytes, in the message data bytes */
		      data[0] = (BYTE) ((AppArrSz & 0x00FF) >> 0);
		      data[1] = (BYTE) ((AppArrSz & 0xFF00) >> 8);
		      *segmented = TRUE;
		    }
		}
	      else
		{
		  /* There's nothing to read... */
		  sdo_error = SDO_ECODE_ATTRIBUTE;
		}
	      break;
	    /*
	    case 2:
	      data[0] = (BYTE) ((AppArrSz & 0x00FF) >> 0);
	      data[1] = (BYTE) ((AppArrSz & 0xFF00) >> 8);
	      *nbytes = 2;
	      break;
	    case 3:
	      data[0] = (BYTE) ((AppArrIndex & 0x00FF) >> 0);
	      data[1] = (BYTE) ((AppArrIndex & 0xFF00) >> 8);
	      *nbytes = 2;
	      break;
	    */
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

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

BYTE app_sdo_read_seg( BYTE od_index_hi,
		       BYTE od_index_lo,
		       BYTE od_subind,
		       BYTE data[7],
		       BYTE *nbytes,
		       BOOL first_segment )
{
  /* Data returned is stored in 'data[]' (up to 7 bytes);
     the number of significant bytes is returned as '*nbytes';
     the return value of the function is the SDO error code */

  BYTE sdo_error = SDO_ECODE_OKAY; /* No error */

  /* Reset array index if necessary */
  if( first_segment ) AppArrIndex = (UINT16) 0;

  /* Read the requested object (segmented) */
  switch( od_index_hi )
    {
    case OD_APP_ARR_HI:
      switch( od_index_lo )
	{
	case OD_APP_ARR_LO:
	  switch( od_subind )
	    {
	    case 0:
	    case 1:
	      /* Read (a segment of 7 bytes of) the example
		 data byte array of arbitrary length */
	      {
		BYTE i;
		if( (AppArrSz - AppArrIndex) > (UINT16) 7 )
		  *nbytes = 7;
		else
		  *nbytes = (BYTE) ((AppArrSz-AppArrIndex) & 0xFF);

		/* Copy a segment of the array to the message data bytes,
		   and add 'od_subind' to each byte, just for fun... */
		for( i=0; i<*nbytes; ++i, ++AppArrIndex )
		  data[i] = AppArr[AppArrIndex] + od_subind;
	      }
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

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

BYTE app_sdo_write_exp( BYTE od_index_hi,
			BYTE od_index_lo,
			BYTE od_subind,
			BYTE data[4],
			BYTE nbytes )
{
  /* Data to be written is stored in 'data[]' (up to 4 bytes);
     'nbytes' is the number of significant bytes;
     the return value of the function is the SDO error code */

  BYTE sdo_error = SDO_ECODE_OKAY; /* No error */

  /* Write the requested object */
  switch( od_index_hi )
    {
    case OD_APP_HI:
      switch( od_index_lo )
	{
	case OD_APP_LO:
	  switch( od_subind )
	    {
	    case 1:
	    case 2:
	      /* The SDO data size is either indicated or not */
	      if( nbytes == 1 || nbytes == 0 )
		{
		  /* Set some parameter */
		  if( app_set_par( od_subind, data ) == FALSE )
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

    case OD_APP_ARR_HI:
      /* Expedited Write -of up to 4 bytes- to the example data byte array */
      switch( od_index_lo )
	{
	case OD_APP_ARR_LO:
	  switch( od_subind )
	    {
	    case 0:
	      {
		BYTE i;
		/* Reset array index */
		AppArrIndex = (UINT16) 0;

		for( i=0; i<nbytes; ++i )
		  {
		    AppArr[AppArrIndex] = data[i];
		    ++AppArrIndex;
		  }
		AppArrSz = AppArrIndex;
	      }
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

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

BYTE app_sdo_write_seg_init( BYTE od_index_hi,
			     BYTE od_index_lo,
			     BYTE od_subind,
			     UINT16 nbytes )
{
  BYTE sdo_error = SDO_ECODE_OKAY; /* No error */

  /* Handle request for a Segmented SDO Download */
  switch( od_index_hi )
    {
    case OD_APP_ARR_HI:
      switch( od_index_lo )
	{
	case OD_APP_ARR_LO:
	  if( od_subind == 0 )
	    {
	      /* Is it going to fit ? */
	      if( nbytes > APP_ARR_SZ_MAX )
		{
		  /* Data block too large */
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

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

BYTE app_sdo_write_seg( BYTE od_index_hi,
			BYTE od_index_lo,
			BYTE od_subind,
			BYTE data[7],
			BYTE nbytes,
			BOOL first_segment )
{
  /* Data to be written is stored in 'data[]' (up to 7 bytes);
     'nbytes' is the number of significant bytes;
     the return value of the function is the SDO error code;
     'first_segment' indicates whether this is the first segment
     of the Segmented Download */

  BYTE sdo_error = SDO_ECODE_OKAY; /* No error */

  /* Reset array index if necessary */
  if( first_segment ) AppArrIndex = (UINT16) 0;

  /* Write the requested object */
  switch( od_index_hi )
    {
    case OD_APP_ARR_HI:
      switch( od_index_lo )
	{
	case OD_APP_ARR_LO:
	  if( od_subind == 0 )
	    {
	      BYTE i;
	      for( i=0; i<nbytes; ++i )
		{
		  if( AppArrIndex < APP_ARR_SZ_MAX )
		    {
		      AppArr[AppArrIndex] = data[i];
		      ++AppArrIndex;
		    }
		  else
		    {
		      /* More bytes don't fit in the array */
		      sdo_error = SDO_ECODE_TYPE_CONFLICT;
		      break; /* Break out of for-loop */
		    }
		}
	      AppArrSz = AppArrIndex;
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

    default:
      /* The index can not be accessed, does not exist */
      sdo_error = SDO_ECODE_NONEXISTENT;
      break;
    }
  return sdo_error;
}

/* ------------------------------------------------------------------------ */

static BOOL app_get_par( BYTE index, BYTE *data, BYTE *no_of_bytes )
{
  /* Called by the SDO server to read something from
     the Object Dictionary which applies to your hardware */

  /* Return TRUE if all went well */
  BYTE result = TRUE;

  /* ...fill in.... */
  switch( index )
    {
    case 1:
#ifdef _VARS_IN_EEPROM_
      /* Refresh variable with copy in EEPROM */
      AppChans = eeprom_read( EE_APP_CHANS );
#endif
      data[0] = AppChans;
      *no_of_bytes = 1;
      break;
    default:
      result = FALSE;
    }

  return result;
}

/* ------------------------------------------------------------------------ */

static BOOL app_set_par( BYTE index, BYTE *data )
{
  /* Called by the SDO server to write something to
     the Object Dictionary which applies to your hardware */

  /* Return TRUE if all went well */
  BYTE result = TRUE;

  /* ...fill in.... */
  switch( index )
    {
    case 1:
      AppChans = data[0];
      break;
    default:
      result = FALSE;
    }

#ifdef _VARS_IN_EEPROM_
  /* Update the working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_APP_CHANS ) != AppChans )
    eeprom_write( EE_APP_CHANS, AppChans );
  /* ...etc...etc..... */
#endif /* _VARS_IN_EEPROM_ */

  return result;
}

/* ------------------------------------------------------------------------ */

/* Up to 16 bytes of configuration parameters can be stored */
#define APP_STORE_SIZE 1

/* ------------------------------------------------------------------------ */

BOOL app_store_config( void )
{
  BYTE block[APP_STORE_SIZE];

  block[0] = AppChans;
  /* ...etc...etc..... */

  return( storage_write_block( STORE_APP, APP_STORE_SIZE, block ) );
}

/* ------------------------------------------------------------------------ */

static void app_load_config( void )
{
  BYTE block[APP_STORE_SIZE];

  /* Read the configuration from EEPROM, if any
     (errors in reading this datablock are caught and
      reported by functions in store.c...) */
  if( storage_read_block( STORE_APP, APP_STORE_SIZE, block ) )
    {
      AppChans = block[0];
      /* ...etc...etc..... */
    }
  else
    {
      /* No valid parameters in EEPROM: use defaults */
      AppChans = APP_DFLT_NO_OF_CHANS;
      /* ...etc...etc..... */
    }

#ifdef _VARS_IN_EEPROM_
  /* Create working copies of configuration globals in EEPROM */
  if( eeprom_read( EE_APP_CHANS ) != AppChans )
    eeprom_write( EE_APP_CHANS, AppChans );
  /* ...etc...etc..... */
#endif /* _VARS_IN_EEPROM_ */
}

/* ------------------------------------------------------------------------ */
