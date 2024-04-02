OBJSX= 

SPB = FrameDecoderGAM.x FrameEncoderGAM.x UTCTimestampingGAM.x DownsamplingGAM.x InvertedPendulumGAM.x

ROOT_DIR=../../..
PACKAGE=Components

MAKEDEFAULTDIR=$(MARTe2_DIR)/MakeDefaults

include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

LIBRARIES_STATIC=$(BUILD_DIR)/FrameEncoderGAM/FrameEncoderGAM$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/FrameDecoderGAM/FrameDecoderGAM$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/UTCTimestampingGAM/UTCTimestampingGAM$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/DownsamplingGAM/DownsamplingGAM$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/InvertedPendulumGAM/InvertedPendulumGAM$(LIBEXT)

all: $(OBJS)  $(SUBPROJ)  \
    $(BUILD_DIR)/GAMs$(LIBEXT) \
    $(BUILD_DIR)/GAMs$(DLLEXT)
	echo  $(OBJS)

include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)
