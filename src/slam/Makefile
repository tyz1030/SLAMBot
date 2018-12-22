include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_LCM) \
	$(CFLAGS_GTK) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GSL) \
	-O3

LDFLAGS = $(LDFLAGS_STD) \
      $(LDFLAGS_MATH) \
      $(LDFLAGS_COMMON) \
      $(LDFLAGS_LCMTYPES) \
      $(LDFLAGS_LCM) \
      $(LDFLAGS_GSL) \
      $(LDFLAGS_MAPPING)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_MAPPING = $(LIB_PATH)/libmapping.a
LIBMAPPING_OBJS = occupancy_grid.o

LIB_SENSOR = $(LIB_PATH)/libsensor.a
LIBSENSOR_OBJS = sensor_model.o moving_laser_scan.o

LIB_ACTION = $(LIB_PATH)/libaction.a
LIBACTION_OBJS = action_model.o

BIN_SLAM = $(BIN_PATH)/slam

ALL = $(LIB_MAPPING) $(LIB_SENSOR) $(BIN_SLAM) $(LIB_ACTION)

all: $(ALL)

$(BIN_SLAM): slam_main.o moving_laser_scan.o mapping.o slam.o action_model.o particle_filter.o sensor_model.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(LIB_MAPPING): $(LIBMAPPING_OBJS)
	@echo "    $@"
	@ar rc $@ $^

$(LIB_SENSOR): $(LIBSENSOR_OBJS)
	@echo "    $@"
	@ar rc $@ $^

$(LIB_ACTION): $(LIBACTION_OBJS) $(LIBDEPS)
	@echo "    $@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
