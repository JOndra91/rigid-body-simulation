BUILDDIR=build
BUILDDIR_RELEASE=build-release
BINDIR=bin

RM=rm -rf
MKDIR=mkdir

first: debug

.PHONY: release
release: CMakeLists.txt qu3e src | $(BUILDDIR_RELEASE)
	cd $(BUILDDIR_RELEASE); make -j

.PHONY: debug
debug: CMakeLists.txt qu3e src | $(BUILDDIR)
	cd $(BUILDDIR); make -j

$(BUILDDIR):
	$(MKDIR) $@
	cd $@; cmake -DCMAKE_BUILD_TYPE=Debug ..

$(BUILDDIR_RELEASE):
	$(MKDIR) $@
	cd $@; cmake -DCMAKE_BUILD_TYPE=Release ..

clean:
	$(RM) $(BUILDDIR) $(BINDIR)
