IMAGES=$(patsubst %.svg,%.pdf, $(wildcard img/*.svg))

doc: $(IMAGES)
	$(MAKE) -B gmu.pdf
	# $(MAKE) -B gmu.bbl
	# $(MAKE) -B gmu.pdf
	# $(MAKE) -B gmu.pdf

pres:
	$(MAKE) -B prezentace.pdf
	$(MAKE) -B prezentace.pdf


include Makefile.mk
