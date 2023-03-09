# HaDiKo Humidity Alarm

It is basically just a temperature and humidity sensoring device, but it has to fulfill multiple requirements:

- Monitoring the air humidity in a predefined interval, e.g. every `30 s`.
- If the humidity exceeds a given threshold, e.g. `65 %`, the device enters a vigilant state. During a set amount of time, the humidity level has to exceed the aforementioned threshold. If it drops below it, control goes back to standby. This measure is thought to prevent false alarms.
- Otherwise, the machine enters the alarm state. This consists of a piezo buzzer that sounds for e.g. `20 s`, then the piezo buzzer shuts off and turns back on after e.g. `5 min`. This alternates for a certain number of times before the alarm remains silent because, apparently, nobody is in that room in order to ventilate it. This is useful during the night where we don't want the alarm to keep ringing until the morning.
- The alarm ends after the room humidity level drops below a separate threshold that can differ from the first one, e.g. `60 %`.
- The device is powered by 3 AAA batteries and is operational over most of their combined voltage range (3.2 to 5 V). However, very low voltages below 3.2 V (this voltage must be considered under load rather than open circuit) affect the humidity sensor in use (AM2302) in a way that an increased measurement error is introduced. In order to avoid false alarms due to low battery, a simple voltage divider leading to an ADC pin has been implemented. The divided battery voltage is then being quantized against the interior ~1.1V voltage reference and the resulting ADC value is converted back to the battery voltage. Once the battery voltage falls below a given threshold, a permanent flag `battOk` is set that prevents any alarm from going off until the device is reset. On device startup, the battery voltage is measured to check the batteries' charge.
- The device also comes with a display that shows the currently measured values, a low battery state and, in case of an ongoing alarm, also displays an alarm message.
- A push-button switch has also been implemented that fulfills multiple purposes:
1) It turns off the piezo buzzer, if on, momentarily until the next alarm cycle begins.
2) It launches a temperature/humidity measurement.
3) It launches an ADC measurement of the battery voltage.
4) It turns on the display in order to display the measurements.

### Energy consumption

Huge considerations have gone to make the device as energy efficient as possible. This, in turn, required special steps:
- The microcontroller in use is an `Atmel ATmega 328P`. It is being used bare-bones, with its internal `8 MHz` clock source, and is directly powered by the batteries (see above).
- Between measurements, the microcontroller is put to sleep in its deepest sleep mode: `SLEEP_MODE_PWR_DOWN`. It is being woken up after its set sleeping time by the Watchdog or after an external `INT0` interrupt that the push-button switch is connected to. The interrupt source is then identified and the device acts accordingly.
- Because the display is the single largest current consumer in the device, it can't possibly remain on the whole time. It's only powered up after a button press for e.g. `10 s`. Then, it is shut off. However, its standby current is still too high which means that it has to be completely disconnected from the supply power. This is achieved by a GPIO pin in combination with a PNP transistor. The microcontroller only turns the transistor on when the display needs to be powered up. Also, the I2C interface is reset (`TWCR = 0;`) when powering down the screen.
- Similar considerations have been made about the humidity sensor. It only needs to be powered up during measurements. Its current consumption is low, so it can be powered directly from a GPIO pin.
- Both display and sensor have a certain minimum start-up time that needs to be respected after being powered on. Only then they become functional.
- The voltage divider for ADC measurements can't be permanently biased between Vcc and GND. Its 4.3 kΩ total resistance still allows a relatively high current to flow, so it needs to be switched as well. I chose to connect the GND side to a GPIO pin and to toggle it `LOW` during measurements.
- All GPIO output pins that don't permanently have to sink or source current are configured as inputs when not in use.

With these efforts, we managed to reduce the deep-sleep total device current to approx. 4-10 µA which should be enough for several months of battery operation.

### Lessons learned and useful links
- When using this microcontroller in the minimal components configuration, it is important to connect the AVcc pin 20 to a voltage source, e.g. to Vcc, even when it isn't planned to use the ADC. If left floating, the device's current consumption will be high even when put to sleep.
- For this project, the microcontroller was programmed using the ISP port and without a bootloader. If the chip was previously used with an external clock source, its fuses need to be reprogrammed. This was done using an AVRISPmkII programmer and the avrdude command line tool. At some later point, the device refused to be programmed and seemed not to react at all although having already been programmed with the right fuses. After a lot of digging, it was found that the AVRISPmkII seems to remember the speed that it had previously been using to program a chip and needs to be specially instructed to lower it using the `-B` parameter (see [this answer from Stackexchange](https://electronics.stackexchange.com/a/97288)).
- The Arduino framework used needs to be configured for this minimal components configuration that uses the internal clock source. This hardware profile including correct fuse settings can be downloaded at [the official Arduino page](https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoToBreadboard#minimal-circuit-eliminating-the-external-clock).
- If fuses need to be tweaked (e.g. set brownout detection level), there's a [useful online tool](http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p) to calculate fuse settings.