/* ------------------------------------------------------------------------
File   : pdo.h

Descr  : Definitions and declarations for CANopen PDO functions.

History: ..DEC.03; Henk B&B; Definition for ELMB framework, based on
                             existing applications.
--------------------------------------------------------------------------- */

#ifndef PDO_H
#define PDO_H

/* Number of Transmit-PDOs */
#define TPDO_CNT          4

/* Number of Receive-PDOs */
#define RPDO_CNT          4

/* Which PDO is used for what */
#define TPDO_APP_IN       (1-1)
#define RPDO_APP_OUT      (1-1)

/* ------------------------------------------------------------------------ */
/* Globals */

/* For timer-triggered PDO transmissions */
extern BOOL   TPdoOnTimer[];

/* Keeps track of time for the timer-triggered PDO transmissions */
extern UINT16 TPdoTimerCntr[];

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void pdo_init          ( void );
void tpdo_scan         ( void );
void pdo_on_nmt        ( BYTE nmt_request );
void tpdo_on_sync      ( void );
void tpdo_on_rtr       ( BYTE pdo_no );
void rpdo              ( BYTE pdo_no, BYTE dlc, BYTE *can_data );
BOOL pdo_rtr_required  ( void );

BOOL tpdo_get_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );
BOOL rpdo_get_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );

BOOL tpdo_get_mapping  ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );
BOOL rpdo_get_mapping  ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE *nbytes,
			 BYTE *par );

BOOL tpdo_set_comm_par ( BYTE pdo_no,
			 BYTE od_subind,
			 BYTE nbytes,
			 BYTE *par );

BOOL pdo_store_config  ( void );

#endif /* PDO_H */
/* ------------------------------------------------------------------------ */
