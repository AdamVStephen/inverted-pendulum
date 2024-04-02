# GPS configuration 

Prior to using a uBlox M8N GPS module within the MARTe for Industry project, it's necesary to update its configuration to produce the messages that the application needs. The necessary configuration is:

* Enabling the UBX-TM-TP message at a 1 Hz cadence, aligned to UTC time
* The measurement rate (UBX-CFG-RATE) should be set to 1 Hz, with UTC as the time source
* The timepulse frequency (UBX-CFG-TP5) should be set to 1 Hz, aligned to UTC time

See the uBlox [protocol description](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) for more information on the configuration of the device.

The easiest tool with which to configure the GPS module is the [u-center](https://www.u-blox.com/en/product/u-center) software, distributed by uBlox. **Note that you will need u-center 1, not u-center 2.** The `u-center 1` download can be found [here](https://www.u-blox.com/sites/default/files/u-centersetup_v21.09.zip).

`u-center` is a Windows application. Note that after installation, when running the application on Windows 10, I received an error message saying that the Microsoft Visual C++ Runtime 2013 could not be found (specifically, that the `msvcr120.dll` could not be found). If you have this problem, I resolved it by downloading the relevant installer from [here](https://www.microsoft.com/en-ph/download/details.aspx?id=40784). Note that I tried the 64 bit version first (`vcredist_x64.exe`), which did not work, but the 32 bit version (`vcredist_x86.exe`) did, suggesting that `u-center 1` is a 32 bit application.

After successfully installing `u-center`, you'll need to connect your computer to the GPS module. For this, I used a USB->UART converter (specifically, the [FTDI C232HD](https://ftdichip.com/wp-content/uploads/2020/07/DS_C232HD_UART_CABLE.pdf)), which provided a COM port on Windows which `u-center`` could use to connect with the GPS module. It's necessary to connect the 3.3V power, GND, TXD and RxD lines to the corresponding pins on the GPS module (see the FTDI datasheet for which connectors correspond to which function).

Once you've connected the GPS module, you can connect `u-center` by clicking the connection icon on the left of the toolbar, and selecting the appropriate COM port from the drop-down menu.

Having connected to the module, you can use the `View->Packet Console` to view the messages being received. If you are using a GPS module with its default configuration, you should see a selection of the standard NMEA messages being produced (note that I've found it can take several minutes from a cold start for the GPS module to start producing valid GPS fixes - but the messages should be produced regardless).

While it's possible to use the `u-center` configuration view (`View->Configuration View`) in order to poll and update the individual settings of the module, `u-center` also supports batch configuration via a file. A file has been prepared which contains the necessary configuration. To use this, go to `Tools->Receiver Configuration...`, select `u-blox M8/8` from the drop-down menu, and navigate to the configuration file (which you can find with this document in the `UKAEA-Sygensys` repo, named `mfi_ublox_m8_configuration.txt`). Tick the `Store configuration into BBR/Flash` checkbox (see note below), followed by `Transfer file->GNSS`. `u-center` will upload the new configuration to the module. You should now see that the time pulse message (`UBX-TIM-TP`) starts to appear in the packet viewer.

**Note:** Ticking the `Store configuration into BBR/Flash` option should write the new configuration to the module's permanent storage so that it is not lost when power-cycled.
