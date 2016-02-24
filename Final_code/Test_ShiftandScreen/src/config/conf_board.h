/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

//! \note TC module is used in this example.
#define EXAMPLE_TC                 (&AVR32_TC0)
//! \note TC Channel 0 is used.
#define EXAMPLE_TC_CHANNEL         0
//! \note IRQ0 line of channel 0 is used.
#define EXAMPLE_TC_IRQ             AVR32_TC0_IRQ0
//! \note Interrupt priority 0 is used for TC in this example.
#define EXAMPLE_TC_IRQ_PRIORITY    AVR32_INTC_INT0

#endif // CONF_BOARD_H
