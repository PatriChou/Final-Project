	.file	"fir.c"
	.option nopic
	.attribute arch, "rv32i2p0"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
.Ltext0:
	.cfi_sections	.debug_frame
	.file 0 "/home/ubuntu/SoC/caravel-soc_fpga-lab-main_v1/lab-caravel_fir/testbench/counter_la_fir" "fir.c"
	.globl	taps
	.data
	.align	2
	.type	taps, @object
	.size	taps, 44
taps:
	.word	0
	.word	-10
	.word	-9
	.word	23
	.word	56
	.word	63
	.word	56
	.word	23
	.word	-9
	.word	-10
	.word	0
	.globl	outputsignal
	.bss
	.align	2
	.type	outputsignal, @object
	.size	outputsignal, 768
outputsignal:
	.zero	768
	.text
	.align	2
	.type	flush_cpu_icache, @function
flush_cpu_icache:
.LFB21:
	.file 1 "../../firmware/system.h"
	.loc 1 15 1
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	s0,12(sp)
	.cfi_offset 8, -4
	addi	s0,sp,16
	.cfi_def_cfa 8, 0
	.loc 1 26 1
	nop
	lw	s0,12(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 16
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE21:
	.size	flush_cpu_icache, .-flush_cpu_icache
	.align	2
	.type	flush_cpu_dcache, @function
flush_cpu_dcache:
.LFB22:
	.loc 1 29 1
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	s0,12(sp)
	.cfi_offset 8, -4
	addi	s0,sp,16
	.cfi_def_cfa 8, 0
	.loc 1 33 1
	nop
	lw	s0,12(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 16
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE22:
	.size	flush_cpu_dcache, .-flush_cpu_dcache
	.section	.mprjram,"ax",@progbits
	.align	2
	.globl	initfir
	.type	initfir, @function
initfir:
.LFB316:
	.file 2 "fir.c"
	.loc 2 4 61
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	s0,12(sp)
	.cfi_offset 8, -4
	addi	s0,sp,16
	.cfi_def_cfa 8, 0
	.loc 2 10 3
	li	a5,805306368
	addi	a5,a5,16
	.loc 2 10 36
	li	a4,64
	sw	a4,0(a5)
	.loc 2 18 2
	li	a5,805306368
	addi	a5,a5,64
	.loc 2 18 38
	sw	zero,0(a5)
	.loc 2 19 2
	li	a5,805306368
	addi	a5,a5,68
	.loc 2 19 40
	li	a4,-10
	sw	a4,0(a5)
	.loc 2 20 2
	li	a5,805306368
	addi	a5,a5,72
	.loc 2 20 40
	li	a4,-9
	sw	a4,0(a5)
	.loc 2 21 2
	li	a5,805306368
	addi	a5,a5,76
	.loc 2 21 40
	li	a4,23
	sw	a4,0(a5)
	.loc 2 22 2
	li	a5,805306368
	addi	a5,a5,80
	.loc 2 22 40
	li	a4,56
	sw	a4,0(a5)
	.loc 2 23 2
	li	a5,805306368
	addi	a5,a5,84
	.loc 2 23 40
	li	a4,63
	sw	a4,0(a5)
	.loc 2 24 2
	li	a5,805306368
	addi	a5,a5,88
	.loc 2 24 40
	li	a4,56
	sw	a4,0(a5)
	.loc 2 25 2
	li	a5,805306368
	addi	a5,a5,92
	.loc 2 25 40
	li	a4,23
	sw	a4,0(a5)
	.loc 2 26 2
	li	a5,805306368
	addi	a5,a5,96
	.loc 2 26 40
	li	a4,-9
	sw	a4,0(a5)
	.loc 2 27 2
	li	a5,805306368
	addi	a5,a5,100
	.loc 2 27 40
	li	a4,-10
	sw	a4,0(a5)
	.loc 2 28 2
	li	a5,805306368
	addi	a5,a5,104
	.loc 2 28 41
	sw	zero,0(a5)
	.loc 2 42 3
	li	a5,805306368
	.loc 2 42 36
	li	a4,1
	sw	a4,0(a5)
	.loc 2 44 1
	nop
	lw	s0,12(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 16
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE316:
	.size	initfir, .-initfir
	.align	2
	.globl	fir
	.type	fir, @function
fir:
.LFB317:
	.loc 2 46 56
	.cfi_startproc
	addi	sp,sp,-32
	.cfi_def_cfa_offset 32
	sw	ra,28(sp)
	sw	s0,24(sp)
	.cfi_offset 1, -4
	.cfi_offset 8, -8
	addi	s0,sp,32
	.cfi_def_cfa 8, 0
	.loc 2 48 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 48 36
	li	a4,-1421869056
	sw	a4,0(a5)
	.loc 2 51 3
	li	a5,-268423168
	addi	a5,a5,56
	.loc 2 51 56
	sw	zero,0(a5)
	.loc 2 54 59
	li	a5,-268423168
	addi	a4,a5,8
	.loc 2 54 112
	li	a5,0
	sw	a5,0(a4)
	.loc 2 54 3
	li	a4,-268423168
	addi	a4,a4,24
	.loc 2 54 56
	sw	a5,0(a4)
	.loc 2 56 2
	call	initfir
	.loc 2 62 7
	li	a5,1
	sw	a5,-20(s0)
	.loc 2 62 2
	j	.L5
.L6:
	.loc 2 72 4 discriminator 3
	li	a5,805306368
	addi	a5,a5,128
	.loc 2 72 37 discriminator 3
	lw	a4,-20(s0)
	sw	a4,0(a5)
	.loc 2 88 40 discriminator 3
	li	a5,805306368
	addi	a5,a5,132
	lw	a4,0(a5)
	.loc 2 88 4 discriminator 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 88 73 discriminator 3
	slli	a4,a4,16
	.loc 2 88 37 discriminator 3
	sw	a4,0(a5)
	.loc 2 62 19 discriminator 3
	lw	a5,-20(s0)
	addi	a5,a5,1
	sw	a5,-20(s0)
.L5:
	.loc 2 62 12 discriminator 1
	lw	a4,-20(s0)
	li	a5,64
	ble	a4,a5,.L6
	.loc 2 223 12
	li	a5,805306368
	.loc 2 223 9
	lw	a5,0(a5)
	sw	a5,-24(s0)
	.loc 2 225 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 225 36
	li	a4,-1420754944
	sw	a4,0(a5)
	.loc 2 226 2
	nop
	.loc 2 227 1
	lw	ra,28(sp)
	.cfi_restore 1
	lw	s0,24(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 32
	addi	sp,sp,32
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE317:
	.size	fir, .-fir
	.text
.Letext0:
	.file 3 "fir.h"
	.file 4 "/opt/riscv/lib/gcc/riscv32-unknown-elf/12.1.0/include/stdint-gcc.h"
	.section	.debug_info,"",@progbits
.Ldebug_info0:
	.4byte	0x14a
	.2byte	0x5
	.byte	0x1
	.byte	0x4
	.4byte	.Ldebug_abbrev0
	.byte	0x7
	.4byte	.LASF18
	.byte	0x1d
	.4byte	.LASF0
	.4byte	.LASF1
	.4byte	.LLRL0
	.4byte	0
	.4byte	.Ldebug_line0
	.byte	0x1
	.byte	0x1
	.byte	0x6
	.4byte	.LASF2
	.byte	0x1
	.byte	0x2
	.byte	0x5
	.4byte	.LASF3
	.byte	0x1
	.byte	0x4
	.byte	0x5
	.4byte	.LASF4
	.byte	0x1
	.byte	0x8
	.byte	0x5
	.4byte	.LASF5
	.byte	0x1
	.byte	0x1
	.byte	0x8
	.4byte	.LASF6
	.byte	0x1
	.byte	0x2
	.byte	0x7
	.4byte	.LASF7
	.byte	0x8
	.4byte	.LASF19
	.byte	0x4
	.byte	0x34
	.byte	0x1b
	.4byte	0x5c
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF8
	.byte	0x1
	.byte	0x8
	.byte	0x7
	.4byte	.LASF9
	.byte	0x9
	.byte	0x4
	.byte	0x5
	.string	"int"
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF10
	.byte	0x3
	.4byte	0x6a
	.4byte	0x88
	.byte	0x4
	.4byte	0x71
	.byte	0xa
	.byte	0
	.byte	0x5
	.4byte	.LASF11
	.byte	0x19
	.4byte	0x78
	.byte	0x5
	.byte	0x3
	.4byte	taps
	.byte	0x3
	.4byte	0x6a
	.4byte	0xa8
	.byte	0x4
	.4byte	0x71
	.byte	0xbf
	.byte	0
	.byte	0x5
	.4byte	.LASF12
	.byte	0x1a
	.4byte	0x98
	.byte	0x5
	.byte	0x3
	.4byte	outputsignal
	.byte	0xa
	.string	"fir"
	.byte	0x2
	.byte	0x2e
	.byte	0x33
	.4byte	.LFB317
	.4byte	.LFE317-.LFB317
	.byte	0x1
	.byte	0x9c
	.4byte	0xf6
	.byte	0xb
	.string	"i"
	.byte	0x2
	.byte	0x3a
	.byte	0x6
	.4byte	0x6a
	.byte	0x2
	.byte	0x91
	.byte	0x6c
	.byte	0x2
	.4byte	.LASF15
	.byte	0x3b
	.byte	0xb
	.4byte	0x50
	.byte	0xc
	.4byte	.LASF13
	.byte	0x2
	.byte	0x3b
	.byte	0x11
	.4byte	0x50
	.byte	0x2
	.byte	0x91
	.byte	0x68
	.byte	0
	.byte	0xd
	.4byte	.LASF14
	.byte	0x2
	.byte	0x4
	.byte	0x33
	.4byte	.LFB316
	.4byte	.LFE316-.LFB316
	.byte	0x1
	.byte	0x9c
	.4byte	0x12d
	.byte	0xe
	.string	"i"
	.byte	0x2
	.byte	0x6
	.byte	0x6
	.4byte	0x6a
	.byte	0x2
	.4byte	.LASF15
	.byte	0x7
	.byte	0xb
	.4byte	0x50
	.byte	0x2
	.4byte	.LASF13
	.byte	0x7
	.byte	0x11
	.4byte	0x50
	.byte	0
	.byte	0x6
	.4byte	.LASF16
	.byte	0x1c
	.4byte	.LFB22
	.4byte	.LFE22-.LFB22
	.byte	0x1
	.byte	0x9c
	.byte	0x6
	.4byte	.LASF17
	.byte	0xe
	.4byte	.LFB21
	.4byte	.LFE21-.LFB21
	.byte	0x1
	.byte	0x9c
	.byte	0
	.section	.debug_abbrev,"",@progbits
.Ldebug_abbrev0:
	.byte	0x1
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0xe
	.byte	0
	.byte	0
	.byte	0x2
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0x21
	.byte	0x2
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x3
	.byte	0x1
	.byte	0x1
	.byte	0x49
	.byte	0x13
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x4
	.byte	0x21
	.byte	0
	.byte	0x49
	.byte	0x13
	.byte	0x2f
	.byte	0xb
	.byte	0
	.byte	0
	.byte	0x5
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0x21
	.byte	0x3
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0x21
	.byte	0x5
	.byte	0x49
	.byte	0x13
	.byte	0x3f
	.byte	0x19
	.byte	0x2
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0x6
	.byte	0x2e
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0x21
	.byte	0x1
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0x21
	.byte	0x25
	.byte	0x27
	.byte	0x19
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0
	.byte	0
	.byte	0x7
	.byte	0x11
	.byte	0x1
	.byte	0x25
	.byte	0xe
	.byte	0x13
	.byte	0xb
	.byte	0x3
	.byte	0x1f
	.byte	0x1b
	.byte	0x1f
	.byte	0x55
	.byte	0x17
	.byte	0x11
	.byte	0x1
	.byte	0x10
	.byte	0x17
	.byte	0
	.byte	0
	.byte	0x8
	.byte	0x16
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x9
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0x8
	.byte	0
	.byte	0
	.byte	0xa
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7c
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xb
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0xc
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0xd
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xe
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_aranges,"",@progbits
	.4byte	0x2c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	.LFB316
	.4byte	.LFE316-.LFB316
	.4byte	.LFB317
	.4byte	.LFE317-.LFB317
	.4byte	0
	.4byte	0
	.section	.debug_rnglists,"",@progbits
.Ldebug_ranges0:
	.4byte	.Ldebug_ranges3-.Ldebug_ranges2
.Ldebug_ranges2:
	.2byte	0x5
	.byte	0x4
	.byte	0
	.4byte	0
.LLRL0:
	.byte	0x6
	.4byte	.Ltext0
	.4byte	.Letext0
	.byte	0x6
	.4byte	.LFB316
	.4byte	.LFE316
	.byte	0x6
	.4byte	.LFB317
	.4byte	.LFE317
	.byte	0
.Ldebug_ranges3:
	.section	.debug_line,"",@progbits
.Ldebug_line0:
	.section	.debug_str,"MS",@progbits,1
.LASF14:
	.string	"initfir"
.LASF13:
	.string	"Status"
.LASF16:
	.string	"flush_cpu_dcache"
.LASF6:
	.string	"unsigned char"
.LASF8:
	.string	"long unsigned int"
.LASF7:
	.string	"short unsigned int"
.LASF18:
	.string	"GNU C17 12.1.0 -mabi=ilp32 -mtune=rocket -misa-spec=2.2 -march=rv32i -g -ffreestanding"
.LASF10:
	.string	"unsigned int"
.LASF11:
	.string	"taps"
.LASF9:
	.string	"long long unsigned int"
.LASF17:
	.string	"flush_cpu_icache"
.LASF12:
	.string	"outputsignal"
.LASF5:
	.string	"long long int"
.LASF3:
	.string	"short int"
.LASF19:
	.string	"uint32_t"
.LASF4:
	.string	"long int"
.LASF2:
	.string	"signed char"
.LASF15:
	.string	"Mask"
	.section	.debug_line_str,"MS",@progbits,1
.LASF0:
	.string	"fir.c"
.LASF1:
	.string	"/home/ubuntu/SoC/caravel-soc_fpga-lab-main_v1/lab-caravel_fir/testbench/counter_la_fir"
	.ident	"GCC: (g1ea978e3066) 12.1.0"
