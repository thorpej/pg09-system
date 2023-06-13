#GALASM= 	galasm
GALASM= 	galette

#GALASM_FLAGS= 	-c -f -p

JEDECS= 	addr_decode0.jed addr_decode1.jed \
		addr_decode2.jed addr_decode3.jed \
		mem_intf.jed

.SUFFIXES:
.SUFFIXES: .gal .jed

.gal.jed:
	$(GALASM) $(GALASM_FLAGS) $<

all: $(JEDECS)

clean:
	-rm -f *.jed *.chp *.fus *.pin
