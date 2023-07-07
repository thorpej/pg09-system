#GALASM= 	galasm
GALASM= 	galette

GALASM_FLAGS= 	-f -p

JEDECS= 	addr_decode0.jed addr_decode1.jed \
		addr_decode2.jed addr_decode3.jed \
		abus_sel.jed \
		intenc.jed intmux.jed intmux_av.jed \
		mem_intf.jed \
		clksel.jed

.SUFFIXES:
.SUFFIXES: .gal .jed

.gal.jed:
	$(GALASM) $(GALASM_FLAGS) $<

all: $(JEDECS)

clean:
	-rm -f *.jed *.chp *.fus *.pin
