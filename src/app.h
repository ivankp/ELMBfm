/* ------------------------------------------------------------------------
File   : app.h

Descr  : Definitions and declarations of user application functions.

History: ..JAN.03; username; Definition.
--------------------------------------------------------------------------- */

#ifndef APP_H
#define APP_H

/* ------------------------------------------------------------------------ */
/* Defaults and configuration */

#define APP_DFLT_NO_OF_CHANS 4

#define APP_MAX_MAPPED_CNT   2

#define APP_ARR_SZ_MAX       ((UINT16) 512)

/* ------------------------------------------------------------------------ */
/* Function prototypes */

void app_init           ( void );
BYTE app_status         ( BYTE *status );

void app_tpdo1          ( void );
void app_tpdo2          ( void );
void app_tpdo3          ( void );
void app_tpdo4          ( void );

void app_rpdo1          ( BYTE dlc, BYTE *can_data );
void app_rpdo2          ( BYTE dlc, BYTE *can_data );
void app_rpdo3          ( BYTE dlc, BYTE *can_data );
void app_rpdo4          ( BYTE dlc, BYTE *can_data );

void app_tpdo_on_cos    ( void );
void app_tpdo_scan_start( void );
void app_tpdo_scan_stop ( void );
void app_tpdo_scan      ( void );

BYTE app_sdo_read       ( BYTE od_index_hi,
			  BYTE od_index_lo,
			  BYTE od_subind,
			  BYTE data[4],
			  BYTE *nbytes,
			  BOOL *segmented );
BYTE app_sdo_read_seg   ( BYTE od_index_hi,
			  BYTE od_index_lo,
			  BYTE od_subind,
			  BYTE data[7],
			  BYTE *nbytes,
			  BOOL first_segment );
BYTE app_sdo_write_exp  ( BYTE od_index_hi,
			  BYTE od_index_lo,
			  BYTE od_subind,
			  BYTE data[4],
			  BYTE nbytes );
BYTE app_sdo_write_seg  ( BYTE od_index_hi,
			  BYTE od_index_lo,
			  BYTE od_subind,
			  BYTE data[7],
			  BYTE nbytes,
			  BOOL first_segment );
BYTE app_sdo_write_seg_init( BYTE od_index_hi,
			     BYTE od_index_lo,
			     BYTE od_subind,
			     UINT16 nbytes );

BOOL app_store_config   ( void );

#endif /* APP_H */
/* ------------------------------------------------------------------------ */
