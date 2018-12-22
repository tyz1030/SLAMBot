include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O3

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	-O3
	 
LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_LCM) \
	  $(LDFLAGS_MAPPING) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_SENSOR) \
	  $(LDFLAGS_ACTION) \
	  $(LDFLAGS_PLANNING)

LIBDEPS = $(call libdeps, $(LDFLAGS))

BOTGUIDIR = botgui
BOTGUIOBJ = botgui.o botgui_main.o

UTILSDIR = utils
UTILSOBJ = drawing_functions.o vx_gtk_window_base.o vx_utils.o

PRINTPDFDIR = printSensorPdf
PRINTPDFOBJ = print_sensor_pdf.o

PRINTACTDIR = printActiongraph
PRINTACTOBJ = test_action_model.o

VPATH = $(BOTGUIDIR) $(UTILSDIR) $(PRINTPDFDIR) $(PRINTACTDIR)

BIN_BOTGUI = $(BIN_PATH)/botgui
BIN_PRINTPDF = $(BIN_PATH)/print_sensor_pdf
BIN_PRINTACT = $(BIN_PATH)/test_action_model

ALL = $(BIN_CHALLENGE_GUI) $(BIN_BOTGUI) $(BIN_PRINTPDF) $(BIN_PRINTACT)

all: $(ALL)
	
$(BIN_BOTGUI): $(BOTGUIOBJ) $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_CHALLENGE_GUI): $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_PRINTPDF): $(PRINTPDFOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_PRINTACT): $(PRINTACTOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a $(BOTGUIDIR)/*.o $(UTILSDIR)/*.o $(PRINTPDFDIR)/*.o $(PRINTACTDIR)/*.o
	@rm -f $(ALL)
