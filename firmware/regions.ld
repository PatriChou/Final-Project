MEMORY {
	vexriscv_debug : ORIGIN = 0xf00f0000, LENGTH = 0x00000100
	dff : ORIGIN = 0x00000000, LENGTH = 0x00000400
	dff2 : ORIGIN = 0x00000400, LENGTH = 0x00000200
	flash : ORIGIN = 0x10000000, LENGTH = 0x01000000
	mprj : ORIGIN = 0x30000000, LENGTH = 0x00200000
	mprjram : ORIGIN = 0x38000000, LENGTH = 0x00000600
	all_data : ORIGIN = 0x38000600, LENGTH = 0x00000100
	all_sdata : ORIGIN = 0x38000700, LENGTH = 0x00000200
	hk : ORIGIN = 0x26000000, LENGTH = 0x00300000
	csr : ORIGIN = 0xf0000000, LENGTH = 0x00010000
}
