BUILDDIR=build
BINDIR=bin

RM=rm -rf
MKDIR=mkdir

first: $(BINDIR)/gmu

$(BINDIR)/gmu: | $(BUILDDIR)
	cd $(BUILDDIR); make

$(BUILDDIR):
	$(MKDIR) $@
	cd $@; cmake ..

clean:
	$(RM) $(BUILDDIR) $(BINDIR)
