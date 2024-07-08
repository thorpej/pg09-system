#GALASM= 	galasm
GALASM= 	galette

GALASM_FLAGS= 	-f -p

JEDECS= 	addr_decode0.jed addr_decode1.jed \
		abus_sel.jed \
		intmux_av.jed \
		mem_intf.jed \
		clksel.jed

.SUFFIXES:
.SUFFIXES: .gal .jed

.gal.jed:
	$(GALASM) $(GALASM_FLAGS) $<

all: $(JEDECS)

clean:
	-rm -f *.jed *.chp *.fus *.pin
