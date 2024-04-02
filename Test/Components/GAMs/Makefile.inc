SPB    = DownsamplingGAM.x 

PACKAGE=Components
ROOT_DIR=../../..
MAKEDEFAULTDIR=$(MARTe2_DIR)/MakeDefaults

include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

LIBRARIES_STATIC+=$(BUILD_DIR)/DownsamplingGAM/DownsamplingGAMTest$(LIBEXT)

all: $(OBJS) $(SUBPROJ) \
    $(BUILD_DIR)/GAMsTests$(LIBEXT)
	echo  $(OBJS)

#include depends.$(TARGET)

include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)
