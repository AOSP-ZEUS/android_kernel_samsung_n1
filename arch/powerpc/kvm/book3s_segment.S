/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright SUSE Linux Products GmbH 2010
 *
 * Authors: Alexander Graf <agraf@suse.de>
 */

/* Real mode helpers */

#if defined(CONFIG_PPC_BOOK3S_64)

#define GET_SHADOW_VCPU(reg)    \
<<<<<<< HEAD
	addi    reg, r13, PACA_KVM_SVCPU
=======
	mr	reg, r13
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#elif defined(CONFIG_PPC_BOOK3S_32)

#define GET_SHADOW_VCPU(reg)    			\
	tophys(reg, r2);       			\
	lwz     reg, (THREAD + THREAD_KVM_SVCPU)(reg);	\
	tophys(reg, reg)

#endif

/* Disable for nested KVM */
#define USE_QUICK_LAST_INST


/* Get helper functions for subarch specific functionality */

#if defined(CONFIG_PPC_BOOK3S_64)
#include "book3s_64_slb.S"
#elif defined(CONFIG_PPC_BOOK3S_32)
#include "book3s_32_sr.S"
#endif

/******************************************************************************
 *                                                                            *
 *                               Entry code                                   *
 *                                                                            *
 *****************************************************************************/

.global kvmppc_handler_trampoline_enter
kvmppc_handler_trampoline_enter:

	/* Required state:
	 *
	 * MSR = ~IR|DR
	 * R13 = PACA
	 * R1 = host R1
	 * R2 = host R2
	 * R10 = guest MSR
	 * all other volatile GPRS = free
	 * SVCPU[CR] = guest CR
	 * SVCPU[XER] = guest XER
	 * SVCPU[CTR] = guest CTR
	 * SVCPU[LR] = guest LR
	 */

	/* r3 = shadow vcpu */
	GET_SHADOW_VCPU(r3)

<<<<<<< HEAD
=======
	/* Save R1/R2 in the PACA (64-bit) or shadow_vcpu (32-bit) */
	PPC_STL	r1, HSTATE_HOST_R1(r3)
	PPC_STL	r2, HSTATE_HOST_R2(r3)

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* Move SRR0 and SRR1 into the respective regs */
	PPC_LL  r9, SVCPU_PC(r3)
	mtsrr0	r9
	mtsrr1	r10

	/* Activate guest mode, so faults get handled by KVM */
	li	r11, KVM_GUEST_MODE_GUEST
<<<<<<< HEAD
	stb	r11, SVCPU_IN_GUEST(r3)
=======
	stb	r11, HSTATE_IN_GUEST(r3)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Switch to guest segment. This is subarch specific. */
	LOAD_GUEST_SEGMENTS

	/* Enter guest */

<<<<<<< HEAD
	PPC_LL	r4, (SVCPU_CTR)(r3)
	PPC_LL	r5, (SVCPU_LR)(r3)
	lwz	r6, (SVCPU_CR)(r3)
	lwz	r7, (SVCPU_XER)(r3)
=======
	PPC_LL	r4, SVCPU_CTR(r3)
	PPC_LL	r5, SVCPU_LR(r3)
	lwz	r6, SVCPU_CR(r3)
	lwz	r7, SVCPU_XER(r3)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	mtctr	r4
	mtlr	r5
	mtcr	r6
	mtxer	r7

<<<<<<< HEAD
	PPC_LL	r0, (SVCPU_R0)(r3)
	PPC_LL	r1, (SVCPU_R1)(r3)
	PPC_LL	r2, (SVCPU_R2)(r3)
	PPC_LL	r4, (SVCPU_R4)(r3)
	PPC_LL	r5, (SVCPU_R5)(r3)
	PPC_LL	r6, (SVCPU_R6)(r3)
	PPC_LL	r7, (SVCPU_R7)(r3)
	PPC_LL	r8, (SVCPU_R8)(r3)
	PPC_LL	r9, (SVCPU_R9)(r3)
	PPC_LL	r10, (SVCPU_R10)(r3)
	PPC_LL	r11, (SVCPU_R11)(r3)
	PPC_LL	r12, (SVCPU_R12)(r3)
	PPC_LL	r13, (SVCPU_R13)(r3)
=======
	PPC_LL	r0, SVCPU_R0(r3)
	PPC_LL	r1, SVCPU_R1(r3)
	PPC_LL	r2, SVCPU_R2(r3)
	PPC_LL	r4, SVCPU_R4(r3)
	PPC_LL	r5, SVCPU_R5(r3)
	PPC_LL	r6, SVCPU_R6(r3)
	PPC_LL	r7, SVCPU_R7(r3)
	PPC_LL	r8, SVCPU_R8(r3)
	PPC_LL	r9, SVCPU_R9(r3)
	PPC_LL	r10, SVCPU_R10(r3)
	PPC_LL	r11, SVCPU_R11(r3)
	PPC_LL	r12, SVCPU_R12(r3)
	PPC_LL	r13, SVCPU_R13(r3)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	PPC_LL	r3, (SVCPU_R3)(r3)

	RFI
kvmppc_handler_trampoline_enter_end:



/******************************************************************************
 *                                                                            *
 *                               Exit code                                    *
 *                                                                            *
 *****************************************************************************/

.global kvmppc_handler_trampoline_exit
kvmppc_handler_trampoline_exit:

<<<<<<< HEAD
=======
.global kvmppc_interrupt
kvmppc_interrupt:

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* Register usage at this point:
	 *
	 * SPRG_SCRATCH0  = guest R13
	 * R12            = exit handler id
<<<<<<< HEAD
	 * R13            = shadow vcpu - SHADOW_VCPU_OFF [=PACA on PPC64]
	 * SVCPU.SCRATCH0 = guest R12
	 * SVCPU.SCRATCH1 = guest CR
=======
	 * R13            = shadow vcpu (32-bit) or PACA (64-bit)
	 * HSTATE.SCRATCH0 = guest R12
	 * HSTATE.SCRATCH1 = guest CR
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	 *
	 */

	/* Save registers */

<<<<<<< HEAD
	PPC_STL	r0, (SHADOW_VCPU_OFF + SVCPU_R0)(r13)
	PPC_STL	r1, (SHADOW_VCPU_OFF + SVCPU_R1)(r13)
	PPC_STL	r2, (SHADOW_VCPU_OFF + SVCPU_R2)(r13)
	PPC_STL	r3, (SHADOW_VCPU_OFF + SVCPU_R3)(r13)
	PPC_STL	r4, (SHADOW_VCPU_OFF + SVCPU_R4)(r13)
	PPC_STL	r5, (SHADOW_VCPU_OFF + SVCPU_R5)(r13)
	PPC_STL	r6, (SHADOW_VCPU_OFF + SVCPU_R6)(r13)
	PPC_STL	r7, (SHADOW_VCPU_OFF + SVCPU_R7)(r13)
	PPC_STL	r8, (SHADOW_VCPU_OFF + SVCPU_R8)(r13)
	PPC_STL	r9, (SHADOW_VCPU_OFF + SVCPU_R9)(r13)
	PPC_STL	r10, (SHADOW_VCPU_OFF + SVCPU_R10)(r13)
	PPC_STL	r11, (SHADOW_VCPU_OFF + SVCPU_R11)(r13)

	/* Restore R1/R2 so we can handle faults */
	PPC_LL	r1, (SHADOW_VCPU_OFF + SVCPU_HOST_R1)(r13)
	PPC_LL	r2, (SHADOW_VCPU_OFF + SVCPU_HOST_R2)(r13)

	/* Save guest PC and MSR */
=======
	PPC_STL	r0, SVCPU_R0(r13)
	PPC_STL	r1, SVCPU_R1(r13)
	PPC_STL	r2, SVCPU_R2(r13)
	PPC_STL	r3, SVCPU_R3(r13)
	PPC_STL	r4, SVCPU_R4(r13)
	PPC_STL	r5, SVCPU_R5(r13)
	PPC_STL	r6, SVCPU_R6(r13)
	PPC_STL	r7, SVCPU_R7(r13)
	PPC_STL	r8, SVCPU_R8(r13)
	PPC_STL	r9, SVCPU_R9(r13)
	PPC_STL	r10, SVCPU_R10(r13)
	PPC_STL	r11, SVCPU_R11(r13)

	/* Restore R1/R2 so we can handle faults */
	PPC_LL	r1, HSTATE_HOST_R1(r13)
	PPC_LL	r2, HSTATE_HOST_R2(r13)

	/* Save guest PC and MSR */
#ifdef CONFIG_PPC64
BEGIN_FTR_SECTION
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	andi.	r0,r12,0x2
	beq	1f
	mfspr	r3,SPRN_HSRR0
	mfspr	r4,SPRN_HSRR1
	andi.	r12,r12,0x3ffd
	b	2f
<<<<<<< HEAD
1:	mfsrr0	r3
	mfsrr1	r4
2:
	PPC_STL	r3, (SHADOW_VCPU_OFF + SVCPU_PC)(r13)
	PPC_STL	r4, (SHADOW_VCPU_OFF + SVCPU_SHADOW_SRR1)(r13)

	/* Get scratch'ed off registers */
	GET_SCRATCH0(r9)
	PPC_LL	r8, (SHADOW_VCPU_OFF + SVCPU_SCRATCH0)(r13)
	lwz	r7, (SHADOW_VCPU_OFF + SVCPU_SCRATCH1)(r13)

	PPC_STL	r9, (SHADOW_VCPU_OFF + SVCPU_R13)(r13)
	PPC_STL	r8, (SHADOW_VCPU_OFF + SVCPU_R12)(r13)
	stw	r7, (SHADOW_VCPU_OFF + SVCPU_CR)(r13)
=======
END_FTR_SECTION_IFSET(CPU_FTR_HVMODE)
#endif
1:	mfsrr0	r3
	mfsrr1	r4
2:
	PPC_STL	r3, SVCPU_PC(r13)
	PPC_STL	r4, SVCPU_SHADOW_SRR1(r13)

	/* Get scratch'ed off registers */
	GET_SCRATCH0(r9)
	PPC_LL	r8, HSTATE_SCRATCH0(r13)
	lwz	r7, HSTATE_SCRATCH1(r13)

	PPC_STL	r9, SVCPU_R13(r13)
	PPC_STL	r8, SVCPU_R12(r13)
	stw	r7, SVCPU_CR(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Save more register state  */

	mfxer	r5
	mfdar	r6
	mfdsisr	r7
	mfctr	r8
	mflr	r9

<<<<<<< HEAD
	stw	r5, (SHADOW_VCPU_OFF + SVCPU_XER)(r13)
	PPC_STL	r6, (SHADOW_VCPU_OFF + SVCPU_FAULT_DAR)(r13)
	stw	r7, (SHADOW_VCPU_OFF + SVCPU_FAULT_DSISR)(r13)
	PPC_STL	r8, (SHADOW_VCPU_OFF + SVCPU_CTR)(r13)
	PPC_STL	r9, (SHADOW_VCPU_OFF + SVCPU_LR)(r13)
=======
	stw	r5, SVCPU_XER(r13)
	PPC_STL	r6, SVCPU_FAULT_DAR(r13)
	stw	r7, SVCPU_FAULT_DSISR(r13)
	PPC_STL	r8, SVCPU_CTR(r13)
	PPC_STL	r9, SVCPU_LR(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*
	 * In order for us to easily get the last instruction,
	 * we got the #vmexit at, we exploit the fact that the
	 * virtual layout is still the same here, so we can just
	 * ld from the guest's PC address
	 */

	/* We only load the last instruction when it's safe */
	cmpwi	r12, BOOK3S_INTERRUPT_DATA_STORAGE
	beq	ld_last_inst
	cmpwi	r12, BOOK3S_INTERRUPT_PROGRAM
	beq	ld_last_inst
	cmpwi	r12, BOOK3S_INTERRUPT_ALIGNMENT
	beq-	ld_last_inst

	b	no_ld_last_inst

ld_last_inst:
	/* Save off the guest instruction we're at */

	/* In case lwz faults */
	li	r0, KVM_INST_FETCH_FAILED

#ifdef USE_QUICK_LAST_INST

	/* Set guest mode to 'jump over instruction' so if lwz faults
	 * we'll just continue at the next IP. */
	li	r9, KVM_GUEST_MODE_SKIP
<<<<<<< HEAD
	stb	r9, (SHADOW_VCPU_OFF + SVCPU_IN_GUEST)(r13)
=======
	stb	r9, HSTATE_IN_GUEST(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*    1) enable paging for data */
	mfmsr	r9
	ori	r11, r9, MSR_DR			/* Enable paging for data */
	mtmsr	r11
	sync
	/*    2) fetch the instruction */
	lwz	r0, 0(r3)
	/*    3) disable paging again */
	mtmsr	r9
	sync

#endif
<<<<<<< HEAD
	stw	r0, (SHADOW_VCPU_OFF + SVCPU_LAST_INST)(r13)
=======
	stw	r0, SVCPU_LAST_INST(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

no_ld_last_inst:

	/* Unset guest mode */
	li	r9, KVM_GUEST_MODE_NONE
<<<<<<< HEAD
	stb	r9, (SHADOW_VCPU_OFF + SVCPU_IN_GUEST)(r13)
=======
	stb	r9, HSTATE_IN_GUEST(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Switch back to host MMU */
	LOAD_HOST_SEGMENTS

	/* Register usage at this point:
	 *
	 * R1       = host R1
	 * R2       = host R2
	 * R12      = exit handler id
<<<<<<< HEAD
	 * R13      = shadow vcpu - SHADOW_VCPU_OFF [=PACA on PPC64]
=======
	 * R13      = shadow vcpu (32-bit) or PACA (64-bit)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	 * SVCPU.*  = guest *
	 *
	 */

	/* RFI into the highmem handler */
	mfmsr	r7
	ori	r7, r7, MSR_IR|MSR_DR|MSR_RI|MSR_ME	/* Enable paging */
	mtsrr1	r7
	/* Load highmem handler address */
<<<<<<< HEAD
	PPC_LL	r8, (SHADOW_VCPU_OFF + SVCPU_VMHANDLER)(r13)
=======
	PPC_LL	r8, HSTATE_VMHANDLER(r13)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	mtsrr0	r8

	RFI
kvmppc_handler_trampoline_exit_end:
