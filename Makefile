BUILDDIR=build
BINDIR=bin

RM=rm -rf
MKDIR=mkdir

first: $(BINDIR)/rigid_body

.PHONY: $(BINDIR)/rigid_body
$(BINDIR)/rigid_body: CMakeLists.txt qu3e src | $(BUILDDIR)
	cd $(BUILDDIR); make -j

$(BUILDDIR):
	$(MKDIR) $@
	cd $@; cmake -DCMAKE_BUILD_TYPE=Debug ..

clean:
	$(RM) $(BUILDDIR) $(BINDIR)
