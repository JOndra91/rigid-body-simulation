BUILDDIR=build
BINDIR=bin

RM=rm -rf
MKDIR=mkdir

first: $(BINDIR)/gmu

$(BINDIR)/gmu: CMakeLists.txt qu3e src | $(BUILDDIR)
	cd $(BUILDDIR); make

$(BUILDDIR):
	$(MKDIR) $@
	cd $@; cmake -DCMAKE_BUILD_TYPE=Debug ..

clean:
	$(RM) $(BUILDDIR) $(BINDIR)
