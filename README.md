# UKAEA-Sygensys

MARTe2 evolution project shared repository

## Description

This is a prototype application demonstrating the use of MARTe2 to implement an industrial control 
application, as part of a collaborative project between UKAEA and Sygensys.

## Build

First, clone and build both `MARTe2` and `MARTe2-components`. For example, on a Raspberry Pi 4 system

```
$ git clone https://github.com/ukaea/MARTe2.git
$ cd MARTe2
$ git checkout ukaea/rpi4
$ make -f Makefile.rpi4
$ export MARTe2_DIR=$(pwd)
$ cd ..
$ git clone https://github.com/ukaea/MARTe2-components.git
$ cd MARTe2-components
$ git checkout ukaea/rpi4
$ make -f Makefile.rpi4
$ export MARTe2_Components_DIR=$(pwd)
```

**Note:** The above `export` commands will only create the `MARTe2_DIR` and `MARTe2_Components_DIR` environment variables for your current terminal session. To avoid having to create these variables in other sessions, you can make them permanent by adding the appropriate lines to your shell configuration file (e.g. `~/.bashrc`):

```
export MARTe2_DIR=<MARTe2 path>
export MARTe2_Components_DIR=<MARTe2-components path>
```

**Note:** When building the `MARTe2` and `MARTe2-components` on the Raspberry Pi, you may see a build failure at the end
related to `libgtest.a`. This only affects the unit test builds, and can safely be ignored for this
project.

Then build the UKAEA/Sygensys components. Assuming that it is located at the same directory level as 
MARTe2 and MARTe2-components:

```
$ cd ../UKAEA-Sygensys
$ make -f Makefile.rpi4
```

## Run

Having built the application's components, you can now run the application (bear in mind that the
environment variables `MARTe2_DIR` and `MARTe2_Components_DIR` must be set - see above).

### Running a test configuration

To run one of the test configurations in the `Configurations/Tests` directory, run the following commands:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Tests/<name of test file> -s State1 --perf
```

See the relevant test report in the `Tests` folder on the Marte For Industry Sharepoint for the hardware and software configuration which needs to be set up before a given test. Note also the use of the `--perf` option to `Main.sh`: this applies various system and application settings which should improve the overall performance of the application. See `Documentation/RPi400/README.md` for more information about the performance settings and their effects.

### Generic serial example

This example demonstrates the use of the `RaspPiSerial` DataSource to listen on a serial port for 
messages, which are then passed into the MARTe2 application as a signal, which is logged to the 
console.

First, set up a pair of virtual serial ports linked together, using the `socat` utility:

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
... socat[8941] N PTY is /dev/pts/8
... socat[8941] N PTY is /dev/pts/9
.. socat[8941] N starting data transfer loop with FDs [5,5] and [7,7]
```

Edit the configuration file `Configurations/Example-RaspPiSerial.cfg` and set the `Port` of the 
`Serial` DataSource to one of the files created by `socat` e.g.

```
...
Port = "/dev/pts/9"
...
```

Then run the example MARTe2 application in another terminal. Execute the following command:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Example-RaspPiSerial.cfg -s State1
...
```

Finally, send a message of the expected size (see the field `MessageSize` in the configuration file)
to the other serial port of the pair, e.g.:

```
echo "abcdefg" > /dev/pts/8
```

You should see the MARTe2 application emit the corresponding `Buffer` signal (as integers 
corresponding to the ASCII characters of the message):

```
...
[Information - LoggerBroker.cpp:152]: Buffer [0:8]:{ 97 98 99 100 101 102 103 10 0 } 
...
```

### GPS example

The first step, if you haven't done it already, is to configure your GPS module. Instructions on how
to do so can be found in the `Documentation/GPS` directory of this repository.

Connect your GPS module and make a note of the serial port it is connected on (e.g. `/dev/serial0`).
Edit the example GPS configuration file `Configurations/Example-GPS.cfg` and set the `Port` of the 
`GPS` DataSource e.g.:

```
...
Port = "/dev/serial0"
...
```

Then run the example GPS application. Execute the following command:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Example-GPS.cfg -s State1
...
```

After some preamble, you should see the application produce GPS signals on the console:

```
...
[Information - LoggerBroker.cpp:152]: Counter [0:0]:12869
[Information - LoggerBroker.cpp:152]: Time [0:0]:4279065408
[Information - LoggerBroker.cpp:152]: ReceivedByteCount [0:0]:137305
[Information - LoggerBroker.cpp:152]: DiscardedByteCount [0:0]:2665
[Information - LoggerBroker.cpp:152]: ReadErrorCount [0:0]:0
[Information - LoggerBroker.cpp:152]: ValidMessageCount [0:0]:5610
[Information - LoggerBroker.cpp:152]: InvalidMessageCount [0:0]:45
[Information - LoggerBroker.cpp:152]: MessageValid [0:0]:1
[Information - LoggerBroker.cpp:152]: TimeOfWeek [0:0]:142428000
[Information - LoggerBroker.cpp:152]: TimeOfWeekSubMS [0:0]:0
[Information - LoggerBroker.cpp:152]: QErr [0:0]:3695
[Information - LoggerBroker.cpp:152]: Week [0:0]:2198
[Information - LoggerBroker.cpp:152]: Flags [0:0]:11
[Information - LoggerBroker.cpp:152]: RefInfo [0:0]:63
[Information - LoggerBroker.cpp:152]: Counter [0:0]:12871
[Information - LoggerBroker.cpp:152]: Time [0:0]:4281065408
...
```

Note that the GPS can take several minutes to start getting fixes, from a cold start. During this time,
it will not produce `UBX-TIM-TP` messages. This will be reflected in the `MessageValid` signal, which
will be set to zero while no messages are being received.

### STM32 example

Connect the STM32 development board to the Pi via serial port, and note the serial port it is connected 
on e.g. `/dev/serial0`. Run the STM32 application on the development board (see 
https://github.com/AndrewLarkins/UKAEA-Sygensys-STM32 for instructions).

Edit the example STM32 configuration file `Configurations/Example-STM32-FileLogging.cfg` and set 
the `Port` of the `STM32` DataSource e.g.:

```
...
Port = "/dev/serial0"
...
```

If you want to make any changes to the STM32 application, you'll need to connect up the board to
your computer via the micro-USB connector (which connects to the embedded ST-Link debug adapter).
Then run the STM32CubeIDE application as per the instructions at the above link.

You may also need to set the baud rate, depending on what configuration the STM32 application is 
using (this test has successfully been run at rates up to 460800 baud, and STM32 data rates up to
1000 data frames per second). To change the baud and/or data rate of the STM32 application:

* Edit `htim3.Init.Prescaler` in `main.c:MX_TIM3_Init` to set the data rate. For example, a value
  of `10000 - 1` will set a data rate of 1 data frame per second, `100 - 1` will give 100 data frames
  per second, and so forth.
* Edit `huart4.Init.BaudRate` in `main.c:MX_UART4_Init` to set the baud rate

In any event, after any edits, select `Run->Run` from the toolbar to launch the new application on
the STM32 board.

Then run the example STM32 MARTe application on the Pi. Execute the following command:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Example-STM32-FileLogging.cfg -s State1
...
```

After the preamble, no terminal output will be produced by the application. You can watch the data
accumulating into the log file by running e.g.

```
$ tail -f stm32-log.csv
```
