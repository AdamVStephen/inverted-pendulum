OBJSX= 

SPB = RaspPiSpi.x RaspPiSerial.x GPS.x STM32.x EduKitSerial.x

ROOT_DIR=../../..
PACKAGE=Components

MAKEDEFAULTDIR=$(MARTe2_DIR)/MakeDefaults

include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

LIBRARIES_STATIC=$(BUILD_DIR)/GPS/GPS$(LIBEXT)
LIBRARIES_STATIC=$(BUILD_DIR)/STM32/STM32$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/RaspPiSerial/RaspPiSerial$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/EduKitSerial/EduKitSerial$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/RaspPiSpi/RaspPiSpi$(LIBEXT)
#LIBRARIES_STATIC+=$(ROOT_DIR)/Build/$(TARGET)/Components/Lib/DataSourceSignalChecker/DataSourceSignalChecker$(LIBEXT)

all: $(OBJS)  $(SUBPROJ)  \
    $(BUILD_DIR)/DataSources$(LIBEXT) \
    $(BUILD_DIR)/DataSources$(DLLEXT)
	echo  $(OBJS)

include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)
