CXX=g++
LIBS=gl sdl2 glew
LDLIBS=$(shell pkg-config --libs-only-l $(LIBS))
LDFLAGS=$(shell pkg-config --libs-only-L --libs-only-other $(LIBS))
CXXFLAGS=--std=c++11 -g -Wall -DGLM_FORCE_RADIANS $(shell pkg-config --cflags $(LIBS))
BUILDDIR=build
BINDIR=bin
OBJ=$(addprefix $(BUILDDIR)/, )

RM=rm -rf
MKDIR=mkdir

first: $(BINDIR)/rigid-body .clang_complete

$(BINDIR)/rigid-body: $(OBJ) | $(BINDIR)
	$(CXX) $(LDFLAGS) $(CXXFLAGS) $(OBJ) $(LDLIBS) -o $@

$(BUILDDIR)/%.o: src/%.cpp | $(BUILDDIR)
	$(CXX) -c $(CXXFLAGS) -o $@ $<

$(BUILDDIR):
	$(MKDIR) $@

$(BINDIR):
	$(MKDIR) $@

.clang_complete: Makefile
	$(shell pkg-config --cflags $(LIBS) | tr ' ' '\n' > $@)

clean:
	$(RM) $(BUILDDIR) $(BINDIR) .clang_complete
