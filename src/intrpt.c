/* ------------------------------------------------------------------------
File   : intrpt.c

Descr  : Empty interrupt routine for all unused interrupts on the
         ELMB Master-processor (ATmega), to handle any spurious interrupts.

History: 22JUN.00; Henk B&B; Definition.
         21JAN.03; Henk B&B; Additions for ATmega128.
--------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------ */

/* Intrpt #1 in use: RESET */
#pragma interrupt_handler empty_handler:2
/* Intrpt #3 in use: INT1 (CAN-controller) */
#pragma interrupt_handler empty_handler:4
#pragma interrupt_handler empty_handler:5
#pragma interrupt_handler empty_handler:6
#pragma interrupt_handler empty_handler:7
#pragma interrupt_handler empty_handler:8
#pragma interrupt_handler empty_handler:9
#pragma interrupt_handler empty_handler:10
/* Intrpt #11 in use: TIMER2 OVF */
#pragma interrupt_handler empty_handler:12
#pragma interrupt_handler empty_handler:13
#pragma interrupt_handler empty_handler:14
/* Intrpt #15 in use: TIMER1 OVF */
#pragma interrupt_handler empty_handler:16
#pragma interrupt_handler empty_handler:17
#pragma interrupt_handler empty_handler:18
#pragma interrupt_handler empty_handler:19
#pragma interrupt_handler empty_handler:20
#pragma interrupt_handler empty_handler:21
#pragma interrupt_handler empty_handler:22
#pragma interrupt_handler empty_handler:23
#pragma interrupt_handler empty_handler:24

#ifndef _ELMB103_
#pragma interrupt_handler empty_handler:25
#pragma interrupt_handler empty_handler:26
#pragma interrupt_handler empty_handler:27
#pragma interrupt_handler empty_handler:28
#pragma interrupt_handler empty_handler:29
#pragma interrupt_handler empty_handler:30
#pragma interrupt_handler empty_handler:31
#pragma interrupt_handler empty_handler:32
#pragma interrupt_handler empty_handler:33
#pragma interrupt_handler empty_handler:34
#pragma interrupt_handler empty_handler:35
#endif /* _ELMB103_ */

void empty_handler( void )
{
  /* Nothing... */
}

/* ------------------------------------------------------------------------ */
