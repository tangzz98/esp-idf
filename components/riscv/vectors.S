// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "soc/soc.h"
#include "soc/interrupt_reg.h"
#include "riscv/rvruntime-frames.h"
#include "soc/soc_caps.h"
#include "sdkconfig.h"

#define STORE                   sw
#define LOAD                    lw
#define REGBYTES                4

	.equ SAVE_REGS, 32
	.equ CONTEXT_SIZE, (SAVE_REGS * 4)
	.equ panic_from_exception, xt_unhandled_exception
	.equ panic_from_isr, panicHandler

.macro save_regs
	addi sp, sp, -CONTEXT_SIZE
	sw   ra,  RV_STK_RA(sp)
	sw   tp,  RV_STK_TP(sp)
	sw   t0,  RV_STK_T0(sp)
	sw   t1,  RV_STK_T1(sp)
	sw   t2,  RV_STK_T2(sp)
	sw   s0,  RV_STK_S0(sp)
	sw   s1,  RV_STK_S1(sp)
	sw   a0,  RV_STK_A0(sp)
	sw   a1,  RV_STK_A1(sp)
	sw   a2,  RV_STK_A2(sp)
	sw   a3,  RV_STK_A3(sp)
	sw   a4,  RV_STK_A4(sp)
	sw   a5,  RV_STK_A5(sp)
	sw   a6,  RV_STK_A6(sp)
	sw   a7,  RV_STK_A7(sp)
	sw   s2,  RV_STK_S2(sp)
	sw   s3,  RV_STK_S3(sp)
	sw   s4,  RV_STK_S4(sp)
	sw   s5,  RV_STK_S5(sp)
	sw   s6,  RV_STK_S6(sp)
	sw   s7,  RV_STK_S7(sp)
	sw   s8,  RV_STK_S8(sp)
	sw   s9,  RV_STK_S9(sp)
	sw   s10, RV_STK_S10(sp)
	sw   s11, RV_STK_S11(sp)
	sw   t3,  RV_STK_T3(sp)
	sw   t4,  RV_STK_T4(sp)
	sw   t5,  RV_STK_T5(sp)
	sw   t6,  RV_STK_T6(sp)
.endm

.macro save_mepc
	csrr t0, mepc
	sw   t0, RV_STK_MEPC(sp)
.endm

.macro restore_regs
	lw   ra,  RV_STK_RA(sp)
	lw   tp,  RV_STK_TP(sp)
	lw   t0,  RV_STK_T0(sp)
	lw   t1,  RV_STK_T1(sp)
	lw   t2,  RV_STK_T2(sp)
	lw   s0,  RV_STK_S0(sp)
	lw   s1,  RV_STK_S1(sp)
	lw   a0,  RV_STK_A0(sp)
	lw   a1,  RV_STK_A1(sp)
	lw   a2,  RV_STK_A2(sp)
	lw   a3,  RV_STK_A3(sp)
	lw   a4,  RV_STK_A4(sp)
	lw   a5,  RV_STK_A5(sp)
	lw   a6,  RV_STK_A6(sp)
	lw   a7,  RV_STK_A7(sp)
	lw   s2,  RV_STK_S2(sp)
	lw   s3,  RV_STK_S3(sp)
	lw   s4,  RV_STK_S4(sp)
	lw   s5,  RV_STK_S5(sp)
	lw   s6,  RV_STK_S6(sp)
	lw   s7,  RV_STK_S7(sp)
	lw   s8,  RV_STK_S8(sp)
	lw   s9,  RV_STK_S9(sp)
	lw   s10, RV_STK_S10(sp)
	lw   s11, RV_STK_S11(sp)
	lw   t3,  RV_STK_T3(sp)
	lw   t4,  RV_STK_T4(sp)
	lw   t5,  RV_STK_T5(sp)
	lw   t6,  RV_STK_T6(sp)
	addi sp, sp, CONTEXT_SIZE
.endm

.macro restore_mepc
	lw	t0, RV_STK_MEPC(sp)
	csrw	mepc, t0
.endm

	.global rtos_int_enter
	.global rtos_int_exit
	.global _global_interrupt_handler

	.section .exception_vectors.text
	/* This is the vector table. MTVEC points here.
	 *
	 * Use 4-byte intructions here. 1 instruction = 1 entry of the table.
	 * The CPU jumps to MTVEC (i.e. the first entry) in case of an exception,
	 * and (MTVEC & 0xfffffffc) + (mcause & 0x7fffffff) * 4, in case of an interrupt.
	 *
	 * Note: for our CPU, we need to place this on a 256-byte boundary, as CPU
	 * only uses the 24 MSBs of the MTVEC, i.e. (MTVEC & 0xffffff00).
	 */

	.balign 0x100
	.global _vector_table
	.type _vector_table, @function
_vector_table:
	.option push
	.option norvc
	j _panic_handler			/* exception handler, entry 0 */
	.rept (ETS_T1_WDT_INUM - 1)
	j _interrupt_handler		/* 24 identical entries, all pointing to the interrupt handler */
	.endr
	j _panic_handler			/* Call panic handler for ETS_T1_WDT_INUM interrupt (soc-level panic)*/
    j _panic_handler			/* Call panic handler for ETS_CACHEERR_INUM interrupt (soc-level panic)*/
    #ifdef CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
    j _panic_handler			/* Call panic handler for ETS_MEMPROT_ERR_INUM interrupt (soc-level panic)*/
	.rept (ETS_MAX_INUM - ETS_MEMPROT_ERR_INUM)
	#else
	.rept (ETS_MAX_INUM - ETS_CACHEERR_INUM)
	#endif
	j _interrupt_handler		/* 6 identical entries, all pointing to the interrupt handler */
	.endr

	.option pop
	.size _vector_table, .-_vector_table

	/* Exception handler.*/
	.type _panic_handler, @function
_panic_handler:
	addi sp, sp, -RV_STK_FRMSZ /* allocate space on stack to store necessary registers */
	/* save general registers */
	sw ra,  RV_STK_RA(sp)
	sw gp,  RV_STK_GP(sp)
	sw tp,  RV_STK_TP(sp)
	sw t0,  RV_STK_T0(sp)
	sw t1,  RV_STK_T1(sp)
	sw t2,  RV_STK_T2(sp)
	sw s0,  RV_STK_S0(sp)
	sw s1,  RV_STK_S1(sp)
	sw a0,  RV_STK_A0(sp)
	sw a1,  RV_STK_A1(sp)
	sw a2,  RV_STK_A2(sp)
	sw a3,  RV_STK_A3(sp)
	sw a4,  RV_STK_A4(sp)
	sw a5,  RV_STK_A5(sp)
	sw a6,  RV_STK_A6(sp)
	sw a7,  RV_STK_A7(sp)
	sw s2,  RV_STK_S2(sp)
	sw s3,  RV_STK_S3(sp)
	sw s4,  RV_STK_S4(sp)
	sw s5,  RV_STK_S5(sp)
	sw s6,  RV_STK_S6(sp)
	sw s7,  RV_STK_S7(sp)
	sw s8,  RV_STK_S8(sp)
	sw s9,  RV_STK_S9(sp)
	sw s10, RV_STK_S10(sp)
	sw s11, RV_STK_S11(sp)
	sw t3,  RV_STK_T3(sp)
	sw t4,  RV_STK_T4(sp)
	sw t5,  RV_STK_T5(sp)
	sw t6,  RV_STK_T6(sp)
	addi t0, sp, RV_STK_FRMSZ /* restore sp with the value when trap happened */
	sw t0,  RV_STK_SP(sp)
	csrr t0, mepc
	sw t0,  RV_STK_MEPC(sp)
	csrr t0, mstatus
	sw t0,  RV_STK_MSTATUS(sp)
	csrr t0, mtvec
	sw t0,  RV_STK_MTVEC(sp)
	csrr t0, mtval
	sw t0,  RV_STK_MTVAL(sp)
	csrr t0, mhartid
	sw t0,  RV_STK_MHARTID(sp)

	/* Call panic_from_exception(sp) or panic_from_isr(sp)
	 * depending on whether we have a pseudo excause or not.
	 * If mcause's highest bit is 1, then an interrupt called this routine,
	 * so we have a pseudo excause. Else, it is due to a exception, we don't
         * have an pseudo excause */
	mv a0, sp
	csrr a1, mcause
	/* Branches instructions don't accept immediates values, so use t1 to
         * store our comparator */
	li t0, 0x80000000
	bgeu a1, t0, _call_panic_handler
	sw a1,  RV_STK_MCAUSE(sp)
	/* exception_from_panic never returns */
	j panic_from_exception
_call_panic_handler:
	/* Remove highest bit from mcause (a1) register and save it in the
	 * structure */
	not t0, t0
	and a1, a1, t0
	sw a1, RV_STK_MCAUSE(sp)
	/* exception_from_isr never returns */
	j panic_from_isr
	.size  panic_from_isr, .-panic_from_isr

	/* This is the interrupt handler.
	 * It saves the registers on the stack,
	 * prepares for interrupt nesting,
	 * re-enables the interrupts,
	 * then jumps to the C dispatcher in interrupt.c.
	 */
	.global _interrupt_handler
	.type _interrupt_handler, @function
#ifndef CONFIG_IDF_RTOS_RTTHREAD

_interrupt_handler:
	/* entry */
	save_regs   /* 保存寄存器 */
	save_mepc   /* 保存MEPC */

	/* Before doing anythig preserve the stack pointer */
	/* It will be saved in current TCB, if needed */
	mv a0, sp            /* 保存SP  a0 = sp */
	call rtos_int_enter

	/* Before dispatch c handler, restore interrupt to enable nested intr */
	csrr s1, mcause    /* 保存mcause s1 = mcause */
	csrr s2, mstatus  /* 保存mstatus  s2 = mstatus */

	/* Save the interrupt threshold level 保存中断嵌套层数? */
	la t0, INTERRUPT_CORE0_CPU_INT_THRESH_REG  /* 保存mstatus  t0 = &INTERRUPT_CORE0_CPU_INT_THRESH_REG */
	lw s3, 0(t0)    /*   s3 = mstatus */

	/* Increase interrupt threshold level 增加中断嵌套层数*/
	li t2, 0x7fffffff
	and t1, s1, t2		/* t1 = mcause & mask */
	slli t1, t1, 2 		/* t1 = mcause * 4 */
	la t2, INTC_INT_PRIO_REG(0)
	add t1, t2, t1		/* t1 = INTC_INT_PRIO_REG + 4 * mcause */
	lw t2, 0(t1)		/* t2 = INTC_INT_PRIO_REG[mcause] */
	addi t2, t2, 1		/* t2 = t2 +1 */
	sw t2, 0(t0)		/* INTERRUPT_CORE0_CPU_INT_THRESH_REG = t2 */
	fence

	li t0, 0x8   /* t0 = 8 */
	csrrs t0, mstatus, t0    /*设置状态MIE寄存器，开总中断*/

	#ifdef CONFIG_PM_TRACE
	li      a0, 0       /* = ESP_PM_TRACE_IDLE */
	#if SOC_CPU_CORES_NUM == 1
	li      a1, 0       /* No need to check core ID on single core hardware */
	#else
	csrr    a1, mhartid
	#endif
	la      t0, esp_pm_trace_exit
	jalr    t0          /* absolute jump, avoid the 1 MiB range constraint */
	#endif

	#ifdef CONFIG_PM_ENABLE
	la      t0, esp_pm_impl_isr_hook
	jalr    t0          /* absolute jump, avoid the 1 MiB range constraint */
	#endif

	/* call the C dispatcher */
	mv      a0, sp      /* argument 1, stack pointer */
	mv      a1, s1      /* argument 2, interrupt number (mcause) */
	/* mask off the interrupt flag of mcause   屏幕异常中断*/
	li	    t0, 0x7fffffff
	and     a1, a1, t0
	jal     _global_interrupt_handler

	/* After dispatch c handler, disable interrupt to make freertos make context switch 
	在调用c函数之后，disable 中断让freertos能够做内容切换
	*/

	li t0, 0x8
	csrrc t0, mstatus, t0  /*清状态MIE寄存器 关总中断*/


	/* restore the interrupt threshold level  中断嵌套 */
	la t0, INTERRUPT_CORE0_CPU_INT_THRESH_REG
	sw s3, 0(t0)
	fence

	/* Yield to the next task is needed: */
	mv a0, sp     /* a0 = sp*/
	call rtos_int_exit

	/* The next (or current) stack pointer is returned in a0 */
	mv sp, a0    /* sp = a0*/

	/* restore the rest of the registers */
	csrw mcause, s1  /* mcause = s1 */
	csrw mstatus, s2   /* mstatus = s2 */
	restore_mepc
	restore_regs

	/* exit, this will also re-enable the interrupts */
	mret
	.size  _interrupt_handler, .-_interrupt_handler
#else
_interrupt_handler:
    /* 此时CPU的sp = from_thread->sp */
    /* 注意： 在这里，并没有将mepc的值赋值为from_thread栈中的epc，但后面会赋值 */
    addi sp, sp, -32 * REGBYTES             /* sp = sp - 32 * 4 栈指针向下偏移32个寄存器长度，用来将CPU的寄存器保存到from_thread的栈中*/
    STORE x1,   1 * REGBYTES(sp)            /* 将CPU的x1寄存器，即ra寄存器，保存到from_thread->栈中 */

    li    t0,   0x80                        /* t0 = 0x80 */
    STORE t0,   2 * REGBYTES(sp)            /* mstatus = t0, 即关闭全局中断 */

    /* 将 CPU 的其他寄存器的值，保存到from_thread的任务栈中 */
    STORE x4,   4 * REGBYTES(sp)
    STORE x5,   5 * REGBYTES(sp)
    STORE x6,   6 * REGBYTES(sp)
    STORE x7,   7 * REGBYTES(sp)
    STORE x8,   8 * REGBYTES(sp)
    STORE x9,   9 * REGBYTES(sp)
    STORE x10, 10 * REGBYTES(sp)
    STORE x11, 11 * REGBYTES(sp)
    STORE x12, 12 * REGBYTES(sp)
    STORE x13, 13 * REGBYTES(sp)
    STORE x14, 14 * REGBYTES(sp)
    STORE x15, 15 * REGBYTES(sp)
    STORE x16, 16 * REGBYTES(sp)
    STORE x17, 17 * REGBYTES(sp)
    STORE x18, 18 * REGBYTES(sp)
    STORE x19, 19 * REGBYTES(sp)
    STORE x20, 20 * REGBYTES(sp)
    STORE x21, 21 * REGBYTES(sp)
    STORE x22, 22 * REGBYTES(sp)
    STORE x23, 23 * REGBYTES(sp)
    STORE x24, 24 * REGBYTES(sp)
    STORE x25, 25 * REGBYTES(sp)
    STORE x26, 26 * REGBYTES(sp)
    STORE x27, 27 * REGBYTES(sp)
    STORE x28, 28 * REGBYTES(sp)
    STORE x29, 29 * REGBYTES(sp)
    STORE x30, 30 * REGBYTES(sp)
    STORE x31, 31 * REGBYTES(sp)

    /* 备份 CPU 的 sp (这时，CPU的sp其实就是from thread的sp指针) 寄存器的值到 s0 寄存器中，下面会使用s0，恢复 CPU 的寄存器 */
    move  s0, sp    /* s0 = sp */

    /* 在中断函数中，中断函数中调用的C函数，需要使用 sp， 这里，在中断函数中，使用的 sp 为，系统的栈资源 */
    /* switch to interrupt stack */
    la    sp, __stack_end__   /* sp = _sp */

    /* interrupt handle */
    /* 注意： 在调用C函数之前,比如sp的值为0x30001000, 在执行完C函数后，sp的值还是会变成 0x30001000 */
    call  rt_interrupt_enter    /* 执行所有的中断函数前，调用该函数 */

    csrr s1, mcause
	csrr s2, mstatus

    /* Save the interrupt threshold level */
	la t0, INTERRUPT_CORE0_CPU_INT_THRESH_REG
	lw s3, 0(t0)

    li t2, 0x7fffffff
	and t1, s1, t2		/* t1 = mcause & mask */
	slli t1, t1, 2 		/* t1 = mcause * 4 */
	la t2, INTC_INT_PRIO_REG(0)
	add t1, t2, t1		/* t1 = INTC_INT_PRIO_REG + 4 * mcause */
	lw t2, 0(t1)		/* t2 = INTC_INT_PRIO_REG[mcause] */
	addi t2, t2, 1		/* t2 = t2 +1 */
	sw t2, 0(t0)		/* INTERRUPT_CORE0_CPU_INT_THRESH_REG = t2 */
	fence

    li t0, 0x8
	csrrs t0, mstatus, t0

    /* call the C dispatcher */
	mv      a0, sp      /* argument 1, stack pointer */
	mv      a1, s1      /* argument 2, interrupt number (mcause) */
	/* mask off the interrupt flag of mcause */
	li	    t0, 0x7fffffff
	and     a1, a1, t0
	jal     _global_interrupt_handler

    li t0, 0x8
	csrrc t0, mstatus, t0

	/* restore the interrupt threshold level */
	la t0, INTERRUPT_CORE0_CPU_INT_THRESH_REG
	sw s3, 0(t0)
	fence

    call  rt_interrupt_leave    /* 执行所有的中断函数后，调用该函数 */

    /* 上面，将保存执行中断服务函数之前的CPU的sp寄存器到了s0所指向的位置处，当执行完中断服务函数，需要将之前的CPU寄存器，恢复一下,此时sp又变成了from thread的sp了 */
    move  sp, s0    /* sp = s0 */

    /* 下面两句话，相当于将 rt_thread_switch_interrupt_flag 值，赋值给了s2  */
    /* 将 rt_thread_switch_interrupt_flag 的地址值，赋值给 s0 寄存器*/
    la    s0, rt_thread_switch_interrupt_flag       /* s0 = &rt_thread_switch_interrupt_flag */
    /* 将 s0 所指向的地址处的内容，取出来，赋值给 s2 寄存器，其实就是将  rt_thread_switch_interrupt_flag 的值，赋值给了 s2 寄存器*/
    lw    s2, 0(s0)                 /* s2 = *s0 = rt_thread_switch_interrupt_flag */

    /* 如果 s2的值，即 rt_thread_switch_interrupt_flag 值，如果不为0，则需要继续执行下一条指令，如果为0，则需要跳转到 spurious_interrupt 标号处 执行 */
    /* 如果 s2的值等于0，rt_thread_switch_interrupt_flag等于0， 则不需要在中断处理函数中，进行上下文切换，反之则需要 */
    /* 如果不需要上下文切换， */

    /* 在这里，跳转到 spurious_interrupt的话，是不会进行上下文切换的，因为，此时CPU的sp指针还是from线程的*/
    beqz  s2, spurious_interrupt    /* if (s2 == 0) goto spurious_interrupt; else 执行下一条语句*/

    /* 需要上下文切换： 主要目的是将CPU的sp指针，赋值为to_thread的sp */
    
    /* 将 s0 所执向的地址的内容设置为0， 也就是，将变量 rt_thread_switch_interrupt_flag 赋值为了 0 */
    /* s0存放的值是 rt_thread_switch_interrupt_flag 变量的地址*/
    sw    zero, 0(s0)       /* *s0 = 0; 也就是 rt_thread_switch_interrupt_flag = 0 */
    /* 将 mepc 的值，赋值给 a0 寄存器，mepc 的值是，跳转到中断函数执行之前的 PC 指针 */
    /* 这时的mpec其实，还是from线程，在跳转到中断执行前的一个PC地址 */
    csrr  a0, mepc  /* a0 = mepc */

    /* 将 mpec 的值写回到freom thread任务栈中的 epc 中,待后续，恢复from线程时，使用 */
    STORE a0, 0 * REGBYTES(sp)  /* from_thread->sp->epc = a0 ，中断入口处*/

    /* 将from_thread的sp指针，赋值为CPU的sp指针 */
    la    s0, rt_interrupt_from_thread  /* s0 = &rt_interrupt_from_thread 注意： rt_interrupt_from_thread = &(from_thread->sp) */
    LOAD  s1, 0(s0)                     /* s1 = rt_interrupt_from_thread，也就是s1 = &(from_thread->sp) */
    STORE sp, 0(s1)                     /* from_thread->sp = sp*/
   
    /* 接下来，需要开始恢复CPU的sp为to_thread的sp了 */
    la    s0, rt_interrupt_to_thread    /* s0 = &rt_interrupt_to_thread 注意： rt_interrupt_to_thread = &(to_thred->sp)*/
    LOAD  s1, 0(s0)                     /* s1 = rt_interrupt_to_thread, 也就是s1 = &(to_thred->sp) */
    LOAD  sp, 0(s1)                     /* sp = (to_thred->sp)*/

    /* 将CPU的 mepc设置为to_thred的mepc，待中断退出，执行mret指令后，将从该地址开始执行 */
    LOAD  a0,  0 * REGBYTES(sp)         /* a0 = to_thread的mepc的值*/
    csrw  mepc, a0                      /* mepc = a0 */


spurious_interrupt:
    LOAD  x1,   1 * REGBYTES(sp)

    /* Remain in M-mode after mret */
    li    t0, 0x00001800
    csrs  mstatus, t0
    LOAD  t0,   2 * REGBYTES(sp)
    csrs  mstatus, t0

    LOAD  x4,   4 * REGBYTES(sp)
    LOAD  x5,   5 * REGBYTES(sp)
    LOAD  x6,   6 * REGBYTES(sp)
    LOAD  x7,   7 * REGBYTES(sp)
    LOAD  x8,   8 * REGBYTES(sp)
    LOAD  x9,   9 * REGBYTES(sp)
    LOAD  x10, 10 * REGBYTES(sp)
    LOAD  x11, 11 * REGBYTES(sp)
    LOAD  x12, 12 * REGBYTES(sp)
    LOAD  x13, 13 * REGBYTES(sp)
    LOAD  x14, 14 * REGBYTES(sp)
    LOAD  x15, 15 * REGBYTES(sp)
    LOAD  x16, 16 * REGBYTES(sp)
    LOAD  x17, 17 * REGBYTES(sp)
    LOAD  x18, 18 * REGBYTES(sp)
    LOAD  x19, 19 * REGBYTES(sp)
    LOAD  x20, 20 * REGBYTES(sp)
    LOAD  x21, 21 * REGBYTES(sp)
    LOAD  x22, 22 * REGBYTES(sp)
    LOAD  x23, 23 * REGBYTES(sp)
    LOAD  x24, 24 * REGBYTES(sp)
    LOAD  x25, 25 * REGBYTES(sp)
    LOAD  x26, 26 * REGBYTES(sp)
    LOAD  x27, 27 * REGBYTES(sp)
    LOAD  x28, 28 * REGBYTES(sp)
    LOAD  x29, 29 * REGBYTES(sp)
    LOAD  x30, 30 * REGBYTES(sp)
    LOAD  x31, 31 * REGBYTES(sp)

    addi  sp, sp, 32 * REGBYTES
    mret
	.size  _interrupt_handler, .-_interrupt_handler
#endif
