#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss


    .global io_wake
io_wake:
	.long 0

    .global io_index
io_index:
	.long 0

    .global next_edge_temp
next_edge_temp:
	.long 0

    .global last_edge_temp
last_edge_temp:
	.long 0



	.global next_edge_1
next_edge_1:
	.long 0


    .global last_edge_1
last_edge_1:
    .long 0


	.global io_number_1
io_number_1:
	.long 0



	.global next_edge_2
next_edge_2:
	.long 0


    .global last_edge_2
last_edge_2:
    .long 0


	.global io_number_2
io_number_2:
	.long 0

	.global next_edge_3
next_edge_3:
	.long 0


    .global last_edge_3
last_edge_3:
    .long 0


	.global io_number_3
io_number_3:
	.long 0



	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* Load io_number */
    move r1, io_index
	ld r0, r1, 0
    add r0,r0,1
    jumpr io_index_loop_end,0x3,lt
    move r0,0

io_index_loop_end:
    st r0,r1,0
    jumpr io1_label,0,eq
    jumpr io2_label,1,eq
    jumpr io3_label,2,eq
io1_label:
    move r1,next_edge_1
    move r0,next_edge_temp
    st r1,r0,0

    move r1,last_edge_1
    move r0,last_edge_temp
    st r1,r0,0

	move r3, io_number_1
    jump io_label_end
io2_label:
    move r1,next_edge_2
    move r0,next_edge_temp
    st r1,r0,0

    move r1,last_edge_2
    move r0,last_edge_temp
    st r1,r0,0

    move r3, io_number_2
    jump io_label_end
io3_label:
    move r1,next_edge_3
    move r0,next_edge_temp
    st r1,r0,0

    move r1,last_edge_3
    move r0,last_edge_temp
    st r1,r0,0

    move r3, io_number_3
    jump io_label_end
io_label_end:

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

    move r3, next_edge_temp
	ld r3, r3, 0
    ld r3, r3, 0

    move r2, last_edge_temp
	ld r2, r2, 0
    ld r2, r2, 0

	add r3, r2, r3
	and r3, r3, 1
	jump in_want_state, eq

    move r2, next_edge_temp
	ld r2, r2, 0
    ld r2, r2, 0
	add r0, r2, r0
	and r0, r0, 1
	jump flip, eq
    halt

in_want_state:
    move r3, last_edge_temp
    ld r3, r3, 0
	st r0, r3, 0
    halt
flip:
    move r3, last_edge_temp
    ld r3, r3, 0
	st r0, r3, 0

    move r3,io_index
    ld r3,r3,0
    move r2,io_wake
    st r3,r2,0
	jump wake_up
	halt




