IMAGES=$(patsubst %.svg,%.pdf, $(wildcard img/*.svg))

doc: $(IMAGES)
	$(MAKE) -B dokumentace.pdf
	$(MAKE) -B dokumentace.bbl
	$(MAKE) -B dokumentace.pdf
	$(MAKE) -B dokumentace.pdf

pres:
	$(MAKE) -B prezentace.pdf
	$(MAKE) -B prezentace.pdf


include Makefile.mk
