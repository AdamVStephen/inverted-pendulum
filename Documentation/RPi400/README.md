# RPi400 configuration 

As part of using an RPi 400 for the MARTe For Industry application, somce changes to the default 
configuration have to be made. These are documented here.

## Performance

In order to maximise the real-time performance of the MARTe application running on the Pi, the
following settings are advised. Note that the relative importance of these settings with respect
to each other is not well understood, but in the interests of reducing doubt, it's best to run
with all of these if possible. 

The settings are divided into three categories:

* Raspberry Pi boot settings
* Raspberry Pi configuration settings
* Application settings

Note that the Raspberry Pi boot settings below require edits to the system `/boot/cmdline.txt` configuration file, followed by a reboot. 

The Raspberry Pi configuration settings do not require a reboot, so they have been incorporated into the application launcher script `Startup/Main.sh`. In order to apply them, add the option `-p` or `--perf` to the command line when running the launcher. Then the configuration settings do not have to be applied manually. Note that the launcher script assumes that CPU 1 is being used as the isolated CPU for running the application. If the user is using a different isolated CPU, the launcher script will need to be modified accordingly.

The application settings are applied within the code of an application. They are already enabled within the MARTe application, but are listed here for completeness.

### Raspberry Pi boot settings

**Disable WiFi**
Add the line `dtoverlay=disable-wifi` to `/boot/config.txt`.

**Disable Graphical desktop**

Run `sudo raspi-config` at the command line, followed by selecting `System Options->Boot / Auto Login->Console`.

**Disable memory swapping**

Add the option `noswap` to the command line in `/boot/cmdline.txt` (a reboot will be required after
this edit).

**Isolate CPU 1**

This provides a single CPU which will be used exclusively for running the time-critical portion
of the MARTe application.

Add the option `isolcpus=1` to the command line in `/boot/cmdline.txt` (a reboot will be required
after this edit).

### Raspberry Pi configuration settings

**Pinning UART interrupts**

In addition to isolating the time-critical portion of the MARTe application on a dedicated CPU, we also want to assign the UART interrupts from the serial ports to be handled on that
CPU. Determine the PL011 UART interrupt number by running the command:

```
cat /sys/class/tty/ttyAMA0/irq
```

Then pin the interrupts to CPU 1 by running:

```
sudo sh -c "echo 2 > /proc/irq/<irq_num>/smp_affinity"
```
where <irq_num> is the interrupt number determined in the previous step. (Note that we use `2` as it is the number with bit set corresponding to CPU 1).

**Scaling governor**

Set CPU 1's scaling governor to `performance` by running the command:

```
sudo sh -c "echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor"
```

This will prevent any throttling of the CPU frequency by the kernel.

**Real-time scheduling**

Enable real-time scheduling by the kernel by running the command:

```
sudo sh -c 'echo "-1" > /proc/sys/kernel/sched_rt_runtime_us'
```
### Application settings

**Set FIFO scheduling**

To configure the scheduler to performa FIFO schuling on the current thread, add the following to your code:

```
#include <sched.h>
...
    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);
```

**Set CPU affinity**

To configure the kernel to assign the current thread to the isolated CPU 1, add the following to your code:

```
#define _GNU_SOURCE
#include <sched.h>
#include <pthread.h>
...
    cpu_set_t useCPUs;
    CPU_ZERO(&useCPUs);
    CPU_SET(1, &useCPUs);

    if (pthread_setaffinity_np(pthread_self(), sizeof (useCPUs), &useCPUs)) {
        perror("main() pthread_setaffinity_np");
        exit(1);
    }
```

**Set tty low latency**

To configure the serial device for low-latency operation, add the following to your code:

```
#include <sys/ioctl.h>
#include <linux/serial.h>
...
        if (ok) {
            struct serial_struct srl;
            ioctl(serial_fd, TIOCGSERIAL, &srl);
            srl.flags |= ASYNC_LOW_LATENCY;
            ioctl(serial_fd, TIOCSSERIAL, &srl);
        }
```

## UART configuration

Two UARTs are required, one for the connection to the STM32, and one for the connection to the GPS.

To enable these, edit the file `/boot/config.txt`, and add the following lines:

```
enable_uart=1
dtoverlay=disable-bt
dtoverlay=uart2
```

The first two lines enable the default UART: since internally the default UART is used for 
communication with the Bluetooth module, it's necessary to disable the Bluetooth module in order
for the UART signals to be routed to the GPIO pins avable on the poin header. The last line enables
the second UART.

Additionally, it's possible that one of the UARTs is assigned as the Linux serial console. If the file `/boot/cmdline.txt` contains the option `console=serial0,115200`, remove it.

In oder for a user to access the serial ports when running the MARTe application, it's necessary to add the user to the `tty` and `dialout` groups. Run the following commands:

```
sudo adduser pi tty
sudo adduser pi dialout
```

Now reboot the Pi. The UART signals should now be accessible, and the `pi` user should be able to read from and write to them.

The pin assignments for UART 0 and UART 1 on the Raspberry Pi 400 are:

| UART        | TXD pin     | RXD pin |
| ----------- | ----------- | ----------- |
| 0           | GPIO 14     | GPIO 15 |
| 1           | GPIO 27     | GPIO 28 |



You may find it helpful to refer to the Pi400 pin header diagram at 
[https://pinout.xyz/pinout/uart](https://pinout.xyz/pinout/uart) for the locations of the 
above GPIO pins on the RPi 400 pin header.
