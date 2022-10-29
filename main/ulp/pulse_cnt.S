#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global next_edge_1
next_edge_1:
	.long 0




    .global last_edge_1
last_edge_1:
    .long 0



	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number_1
io_number_1:
	.long 0




	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* Load io_number */
	move r3, io_number_1
	ld r3, r3, 0
	move r0, r3
	jumpr read_io_high, 16, ge

	/* Read the value of lower 16 RTC IOs into R0 */
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
	rsh r0, r0, r3
	jump read_done


	/* Read the value of RTC IOs 16-17, into R0 */
read_io_high:
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2)
	sub r3, r3, 16
	rsh r0, r0, r3
read_done:
	and r0, r0, 1

    move r3, next_edge_1
	ld r3, r3, 0

    move r2, last_edge_1
	ld r2, r2, 0

	add r3, r2, r3
	and r3, r3, 1
	jump in_want_state, eq

    move r2, next_edge_1
	ld r2, r2, 0
	add r0, r2, r0
	and r0, r0, 1
	jump flip, eq
    halt

in_want_state:
    move r3, last_edge_1
	st r0, r3, 0
    halt
flip:
    move r3, last_edge_1
	st r0, r3, 0
	jump wake_up
	halt




