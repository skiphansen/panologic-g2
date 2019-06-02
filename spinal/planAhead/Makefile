
project:
	planAhead -source planAhead.tcl -mode batch

progmem:
	data2mem -bm ../ise/progmem.bmm -bt planAhead.runs/impl_1/Pano.bit -bd ../sw/progmem.mem -o b planAhead.runs/impl_1/Pano_ram.bit

clean:
	rm -rf planAhead.runs planAhead.data planAhead.srcs *.jou *.log planAhead.ppr planAhead.cache
