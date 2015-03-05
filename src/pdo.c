/* ------------------------------------------------------------------------
File   : pdo.c

Descr  : Functions for CANopen PDO handling, which call node-specific
         device routines.

History: ..DEC.03; Henk B&B; Definition for ELMB framework, based on
                             existing applications.
--------------------------------------------------------------------------- */

#include "general.h"
#include "app.h"
#include "can.h"
#include "eeprom.h"
#include "objects.h"
#include "pdo.h"
#include "store.h"
#include "timer1XX.h"

/* ------------------------------------------------------------------------ */
/* Some PDO communication parameters and the PDO mappings are constant
   in this application, and can thus be stored in program memory;
   this information is here just for reference, it is not essential
   for correct operation, although it can be read out by SDO messages;
   in the data arrays below the RPDO parameters are stored behind
   the TPDO parameters */

/* Per PDO the corresponding COB-ID (default predefined CANopen values..),
   here: TPDO1 to 4 and RPDO1 to 4 */
const UINT16 PDO_COBID[TPDO_CNT+RPDO_CNT] = { 0x180, 0x280, 0x380, 0x480,
					      0x200, 0x300, 0x400, 0x500 };

/* Per PDO the number of mapped objects */
extern const BYTE   PDOMAP_CNT[TPDO_CNT+RPDO_CNT];

/* Per PDO the mapped objects */
extern const UINT32 PDOMAP[TPDO_CNT+RPDO_CNT][APP_MAX_MAPPED_CNT];

/* ------------------------------------------------------------------------ */
/* Globals */

/* Transmit-PDO and Receive-PDO communication parameters,
   stored in one array: first the TPDO parameters, then the RPDO parameters */
static PDO_COMM_PAR PdoCommPar[TPDO_CNT+RPDO_CNT]; /* (copy in EEPROM) */
static PDO_COMM_PAR *TPdoCommPar = &PdoCommPar[0];
static PDO_COMM_PAR *RPdoCommPar = &PdoCommPar[TPDO_CNT];

/* For timer-triggered PDO transmissions */
BOOL                TPdoOnTimer[TPDO_CNT];         /* (copy in EEPROM) */

/* Keeps track of time for the timer-triggered PDO transmissions
   (Timer1 is used to update these counters) */
UINT16              TPdoTimerCntr[TPDO_CNT];

/* ------------------------------------------------------------------------ */
/* Local prototypes */

static void pdo_load_config( void );

static BOOL pdo_get_comm_par( BYTE pdo_no,
			      BYTE od_subind,
			      BYTE *nbytes,
			      BYTE *par );

static BOOL pdo_get_mapping( BYTE pdo_no,
			     BYTE od_subind,
			     BYTE *nbytes,
			     BYTE *par );

/* ------------------------------------------------------------------------ */

void pdo_init( void )
{
  BYTE i;

  /* Initialize PDO configuration parameters */
  pdo_load_config();

#ifdef _VARS_IN_EEPROM_
  for( i=0; i<TPDO_CNT+RPDO_CNT; ++i )
    {
      BYTE byt;
      if( eeprom_read( EE_PDO_TTYPE+i ) != PdoCommPar[i].transmission_type )
	eeprom_write( EE_PDO_TTYPE+i, PdoCommPar[i].transmission_type );
      byt = (BYTE) (PdoCommPar[i].event_timer & (UINT16) 0x00FF);
      if( eeprom_read( EE_PDO_ETIMER_LO+i ) != byt )
	eeprom_write( EE_PDO_ETIMER_LO+i, byt );
      byt = (BYTE) ((PdoCommPar[i].event_timer & (UINT16) 0xFF00) >> 8);
      if( eeprom_read( EE_PDO_ETIMER_HI+i ) != byt )
	eeprom_write( EE_PDO_ETIMER_HI+i, byt );
    }
#endif /* _VARS_IN_EEPROM_ */

  /* Set timer stuff for Transmit-PDOs */
  TIMER1_DISABLE();
  for( i=0; i<TPDO_CNT; ++i )
    {
      TPdoOnTimer[i]    = ((TPdoCommPar[i].transmission_type >= 254) &&
			    (TPdoCommPar[i].event_timer > (UINT16)0));
      TPdoTimerCntr[i]  = (UINT16) 0;

#ifdef _VARS_IN_EEPROM_
      if( eeprom_read( EE_TPDO_ONTIMER+i ) != TPdoOnTimer[i] )
	eeprom_write( EE_TPDO_ONTIMER+i, TPdoOnTimer[i] );
#endif /* _VARS_IN_EEPROM_ */
    }
  TIMER1_ENABLE();

  /* If Remote Frames are not required adjust
     the CAN-controller's configuration */
  can_rtr_enable( pdo_rtr_required() );
}

/* ------------------------------------------------------------------------ */

void tpdo_scan( void )
{
  /* Handle ongoing multi-PDO transmissions (non-standard CANopen...),
     change-of-state PDO transmissions,
     and timer-triggered PDO transmission(s) */
  BYTE pdo_no;

  /* Ongoing multi-channel readout */
  app_tpdo_scan();

  /* PDO(s) to be sent on a change-of-state of the I/O */
  app_tpdo_on_cos();

  /* Timer-triggered Transmit-PDOs */
  for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
    {
#ifdef _VARS_IN_EEPROM_
      TPdoOnTimer[pdo_no] = eeprom_read( EE_TPDO_ONTIMER + pdo_no );
#endif /* _VARS_IN_EEPROM_ */

      if( TPdoOnTimer[pdo_no] )
	{
#ifdef _VARS_IN_EEPROM_
	  TPdoCommPar[pdo_no].event_timer =
	    ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
	    (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* _VARS_IN_EEPROM_ */

	  /* If timer period expired... */ 
	  if( TPdoTimerCntr[pdo_no] >= TPdoCommPar[pdo_no].event_timer )
	    {
	      switch( pdo_no )
		{
		case 0:
		  app_tpdo1();
		  break;
		case 1:
		  app_tpdo2();
		  break;
		case 2:
		  app_tpdo3();
		  break;
		case 3:
		  app_tpdo4();
		  break;
		default:
		  break;
		}

	      TIMER1_DISABLE();
	      TPdoTimerCntr[pdo_no] = (UINT16) 0;
	      TIMER1_ENABLE();
	    }
	}
    }
}

/* ------------------------------------------------------------------------ */

void pdo_on_nmt( BYTE nmt_request )
{
  switch( nmt_request )
    {
    case NMT_START_REMOTE_NODE:
      {
	/* Immediately start first timer-triggered read out, if enabled... */
	BYTE pdo_no;
	TIMER1_DISABLE();
	for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
	  {
#ifdef _VARS_IN_EEPROM_
	    TPdoCommPar[pdo_no].event_timer =
	      ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
	      (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* _VARS_IN_EEPROM_ */

	    TPdoTimerCntr[pdo_no] = TPdoCommPar[pdo_no].event_timer;
	  }
	TIMER1_ENABLE();
      }
      break;

    case NMT_STOP_REMOTE_NODE:
    case NMT_ENTER_PREOPERATIONAL_STATE:
    case NMT_RESET_COMMUNICATION:
    case NMT_RESET_NODE:
      /* Do everything necessary to stop any ongoing
	 multi-channel readout operations properly */
      app_tpdo_scan_stop();

      break;

    default:
      break;
    }
}

/* ------------------------------------------------------------------------ */

void tpdo_on_sync( void )
{
  /* Send Transmit-PDO(s) on the reception of a SYNC object:
     only if Transmit-PDO(s) has (have) appropriate transmission type */
  BYTE pdo_no;
  for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
    {
#ifdef _VARS_IN_EEPROM_
      TPdoCommPar[pdo_no].transmission_type =
	eeprom_read( EE_PDO_TTYPE + pdo_no );
#endif /* _VARS_IN_EEPROM_ */

      /* Only if Transmit-PDO has appropriate transmission type */
      if( TPdoCommPar[pdo_no].transmission_type == 1 )
	{
	  switch( pdo_no )
	    {
	    case 0:
	      app_tpdo1();
	      break;
	    case 1:
	      app_tpdo2();
	      break;
	    case 2:
	      app_tpdo3();
	      break;
	    case 3:
	      app_tpdo4();
	      break;
	    default:
	      break;
	    }
	}
    }
}

/* ------------------------------------------------------------------------ */

void tpdo_on_rtr( BYTE pdo_no )
{
  /* Remote Transmission Request for a Transmit-PDO */

#ifdef _VARS_IN_EEPROM_
  TPdoCommPar[pdo_no].transmission_type = eeprom_read( EE_PDO_TTYPE + pdo_no );
#endif /* _VARS_IN_EEPROM_ */

  /* Only if TPDO has appropriate transmission type */
  if( TPdoCommPar[pdo_no].transmission_type >= 253 )
    {
      switch( pdo_no )
	{
	case 0:
	  app_tpdo1();
	  break;
	case 1:
	  app_tpdo2();
	  break;
	case 2:
	  app_tpdo3();
	  break;
	case 3:
	  app_tpdo4();
	  break;
	default:
	  break;
	}
    }
}

/* ------------------------------------------------------------------------ */

void rpdo( BYTE pdo_no, BYTE dlc, BYTE *can_data )
{
  switch( pdo_no )
    {
    case 0:
      /* Receive-PDO1 */
      app_rpdo1( dlc, can_data );
      break;
    case 1:
      /* Receive-PDO2 */
      app_rpdo2( dlc, can_data );
      break;
    case 2:
      /* Receive-PDO3 */
      app_rpdo3( dlc, can_data );
      break;
    case 3:
      /* Receive-PDO4 */
      app_rpdo4( dlc, can_data );
      break;
    default:
      break;
    }
}

/* ------------------------------------------------------------------------ */

BOOL pdo_rtr_required( void )
{
  /* Check if any of the transmission types requires CAN Remote Frames */
  BOOL pdo_no;
  BOOL required = FALSE;

  for( pdo_no=0; pdo_no<TPDO_CNT; ++pdo_no )
    {
#ifdef _VARS_IN_EEPROM_
      TPdoCommPar[pdo_no].transmission_type =
	eeprom_read( EE_PDO_TTYPE + pdo_no );
#endif /* _VARS_IN_EEPROM_ */

      /* Only if Transmit-PDO has certain transmission types */
      if( TPdoCommPar[pdo_no].transmission_type >= 253 )
	required = TRUE;
    }

  return required;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_get_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE *nbytes,
			BYTE *par )
{
  if( pdo_no < TPDO_CNT )
    {
      return( pdo_get_comm_par( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL rpdo_get_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE *nbytes,
			BYTE *par )
{
  if( pdo_no < RPDO_CNT )
    {
      pdo_no += TPDO_CNT; /* RPDO parameters are stored BEHIND the TPDO pars */
      return( pdo_get_comm_par( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_get_mapping( BYTE pdo_no,
		       BYTE od_subind,
		       BYTE *nbytes,
		       BYTE *par )
{
  if( pdo_no < TPDO_CNT )
    {
      return( pdo_get_mapping( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL rpdo_get_mapping( BYTE pdo_no,
			BYTE od_subind,
			BYTE *nbytes,
			BYTE *par )
{
  if( pdo_no < RPDO_CNT )
    {
      pdo_no += TPDO_CNT; /* RPDO parameters are stored BEHIND the TPDO pars */
      return( pdo_get_mapping( pdo_no, od_subind, nbytes, par ) );
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */

BOOL tpdo_set_comm_par( BYTE pdo_no,
			BYTE od_subind,
			BYTE nbytes,
			BYTE *par )
{
  /* If 'nbytes' is zero it means the data set size was
     not indicated in the SDO message */

  if( pdo_no >= TPDO_CNT ) return FALSE;

  switch( od_subind )
    {
    case OD_PDO_TRANSMTYPE:
      if( nbytes == 1 || nbytes == 0 )
	{
	  TPdoCommPar[pdo_no].transmission_type = par[0];

#ifdef _VARS_IN_EEPROM_
	  if( eeprom_read(EE_PDO_TTYPE+pdo_no) !=
	      TPdoCommPar[pdo_no].transmission_type )
	    eeprom_write( EE_PDO_TTYPE+pdo_no,
			  TPdoCommPar[pdo_no].transmission_type );
#endif /* _VARS_IN_EEPROM_ */

	  /* Adjust CAN-controller configuration if necessary */
	  can_rtr_enable( pdo_rtr_required() );
	}
      else
	return FALSE;
      break;

    case OD_PDO_EVENT_TIMER:
      if( nbytes == 2 || nbytes == 0 )
	{
	  /* In units of seconds, <=65535 */
	  TPdoCommPar[pdo_no].event_timer  = (UINT16) par[0];
	  TPdoCommPar[pdo_no].event_timer |= (((UINT16) par[1]) << 8);

#ifdef _VARS_IN_EEPROM_
	  if( eeprom_read( EE_PDO_ETIMER_LO+pdo_no ) != par[0] )
	    eeprom_write( EE_PDO_ETIMER_LO+pdo_no, par[0] );
	  if( eeprom_read( EE_PDO_ETIMER_HI+pdo_no ) != par[1] )
	    eeprom_write( EE_PDO_ETIMER_HI+pdo_no, par[1] );
#endif /* _VARS_IN_EEPROM_ */
	}
      else
	return FALSE;
      break;

    default:
      /* The sub-index does not exist */
      return FALSE;
    }

#ifdef _VARS_IN_EEPROM_
  TPdoCommPar[pdo_no].transmission_type = eeprom_read(EE_PDO_TTYPE+pdo_no);
  TPdoCommPar[pdo_no].event_timer       =
    ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
    (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* _VARS_IN_EEPROM_ */

  /* Update the PDO Event Timer stuff if necessary */
  TPdoOnTimer[pdo_no] = ((TPdoCommPar[pdo_no].transmission_type >= 254)
			 && (TPdoCommPar[pdo_no].event_timer > (UINT16)0));
#ifdef _VARS_IN_EEPROM_
  if( eeprom_read( EE_TPDO_ONTIMER+pdo_no ) != TPdoOnTimer[pdo_no] )
    eeprom_write( EE_TPDO_ONTIMER+pdo_no, TPdoOnTimer[pdo_no] );
#endif /* _VARS_IN_EEPROM_ */

  /* Immediately start first timer-triggered read out, if enabled... */
  TIMER1_DISABLE();
  TPdoTimerCntr[pdo_no] = TPdoCommPar[pdo_no].event_timer;
  TIMER1_ENABLE();
  
  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL pdo_get_comm_par( BYTE pdo_no,
			      BYTE od_subind,
			      BYTE *nbytes,
			      BYTE *par )
{
  switch( od_subind )
    {
    case OD_NO_OF_ENTRIES:
      par[0]  = 5;
      *nbytes = 1;
      break;

    case OD_PDO_COBID:
      {
	UINT16 cob_id;

#ifdef _VARS_IN_EEPROM_
	NodeID = eeprom_read( EE_NODEID );
#endif /* _VARS_IN_EEPROM_ */

	/* Default values... */
	cob_id = PDO_COBID[pdo_no] | NodeID;

	par[0] = (BYTE) ((cob_id & (UINT16) 0x00FF) >> 0);
	par[1] = (BYTE) ((cob_id & (UINT16) 0xFF00) >> 8);
	par[2] = 0x00;
	par[3] = 0x00;
	*nbytes = 4;
      }
      break;

    case OD_PDO_TRANSMTYPE:

#ifdef _VARS_IN_EEPROM_
      PdoCommPar[pdo_no].transmission_type = eeprom_read(EE_PDO_TTYPE+pdo_no);
#endif /* _VARS_IN_EEPROM_ */

      par[0]  = PdoCommPar[pdo_no].transmission_type;
      *nbytes = 1;
      break;

    case OD_PDO_INHIBITTIME:
      par[0]  = 0x00;
      par[1]  = 0x00;
      *nbytes = 2;
      break;

    case OD_PDO_EVENT_TIMER:
      {
	/* In units of seconds, <=65535 */
#ifdef _VARS_IN_EEPROM_
	PdoCommPar[pdo_no].event_timer       =
	  ((UINT16) eeprom_read( EE_PDO_ETIMER_LO + pdo_no )) |
	  (((UINT16) eeprom_read( EE_PDO_ETIMER_HI + pdo_no )) << 8);
#endif /* _VARS_IN_EEPROM_ */

	par[0] = (BYTE) (PdoCommPar[pdo_no].event_timer & (UINT16) 0x00FF);
	par[1] = (BYTE) ((PdoCommPar[pdo_no].event_timer &
			  (UINT16) 0xFF00) >> 8);
	*nbytes = 2;
      }
      break;

    default:
      /* The sub-index does not exist */
      return FALSE;
    }

  return TRUE;
}

/* ------------------------------------------------------------------------ */

static BOOL pdo_get_mapping( BYTE pdo_no,
			     BYTE od_subind,
			     BYTE *nbytes,
			     BYTE *par )
{
  if( od_subind == OD_NO_OF_ENTRIES )
    {
      par[0] = PDOMAP_CNT[pdo_no];
      *nbytes = 1;
    }
  else
    {
      if( od_subind <= PDOMAP_CNT[pdo_no] )
	{
	  BYTE i;
	  const BYTE *p = (const BYTE *) &PDOMAP[pdo_no][od_subind-1];

	  for( i=0; i<4; ++i, ++p ) par[i] = *p;

	  *nbytes = 4;
	}
      else
	{
	  /* The sub-index does not exist */
	  return FALSE;
	}
    }
  return TRUE;
}

/* ------------------------------------------------------------------------ */

/* Note that not all PDO parameters fit in one storage block (16 bytes max)
   when there are more than 5 PDOs */
#define TPDO_STORE_SIZE (TPDO_CNT * sizeof(PDO_COMM_PAR))
#define RPDO_STORE_SIZE (RPDO_CNT * sizeof(PDO_COMM_PAR))

/* ------------------------------------------------------------------------ */

BOOL pdo_store_config( void )
{
  BYTE *p;
  BOOL result = TRUE;

#ifdef _VARS_IN_EEPROM_
  BYTE i;
  for( i=0; i<TPDO_CNT+RPDO_CNT; ++i )
    {
      PdoCommPar[i].transmission_type = eeprom_read( EE_PDO_TTYPE+i );
      PdoCommPar[i].event_timer       =
	((UINT16) eeprom_read( EE_PDO_ETIMER_LO + i )) |
	(((UINT16) eeprom_read( EE_PDO_ETIMER_HI + i )) << 8);
    }
#endif /* _VARS_IN_EEPROM_ */

  /* Store the configurations in EEPROM */
  p = (BYTE *) TPdoCommPar;
  if( storage_write_block( STORE_TPDO, TPDO_STORE_SIZE, p ) == FALSE )
    result = FALSE;
  p = (BYTE *) RPdoCommPar;
  if( storage_write_block( STORE_RPDO, RPDO_STORE_SIZE, p ) == FALSE )
    result = FALSE;

  return result;
}

/* ------------------------------------------------------------------------ */

static void pdo_load_config( void )
{
  BYTE *p;

  /* Read the configuration from EEPROM, if any */
  p = (BYTE *) TPdoCommPar;
  if( !storage_read_block( STORE_TPDO, TPDO_STORE_SIZE, p ) )
    {
      /* No valid parameters in EEPROM: use defaults */
      BYTE i;

      /* Set default PDO-Transmit communication parameters */
      for( i=0; i<TPDO_CNT; ++i )
	{
	  TPdoCommPar[i].transmission_type = 1;	 /* Respond to SYNC */
	  TPdoCommPar[i].event_timer       = 0;	 /* Seconds between triggers;
						    0 = not timer-triggered */
	}
    }

  /* Read the configuration from EEPROM, if any */
  p = (BYTE *) RPdoCommPar;
  if( !storage_read_block( STORE_RPDO, RPDO_STORE_SIZE, p ) )
    {
      /* No valid parameters in EEPROM: use defaults */
      BYTE i;

      /* Set default PDO-Receive communication parameters */
      for( i=0; i<RPDO_CNT; ++i )
	{
	  RPdoCommPar[i].transmission_type = 255;/* Profile specific */
	  RPdoCommPar[i].event_timer       = 0;	 /* Not used for TPDOs... */
	}
    }
}

/* ------------------------------------------------------------------------ */
