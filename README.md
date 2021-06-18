# ULP-Pulse

Some short examples of using an extended version of the pulse counter from here: https://github.com/espressif/esp-idf/blob/master/examples/system/ulp/main/ulp/pulse_cnt.S

Besides count the pulses it also saves the shortest one. This enables some more use cases.

## Requirements

These examples need an ESP32 and currently only work with Arduino IDE, which has to be extended with "ulptool": https://github.com/duff2013/ulptool

## Wind Speed

First example for the extended pulse counter is the measurement of wind speed and wind gusts. While the ESP32 is in Deep Sleep, the ULP counts the pulses of an Anemometer.
Combined with the timeframe of sleeping, the wind speed can be calculated easily. On top of that, the shortest pulse can be used to calculate the highest wind gust.

```
WINDFACTOR = 2.4     // 2.4 km/h per pulse defined by the Anemometer
TIMEFACTOR = 1000000 // factor between seconds and microseconds
```

### Pulse Count Usage
```C
float wind = (pulses / timeSleep) * WINDFACTOR
```

### Shortest Pulse Usage
```C
float windGust = (1 / (shortestPulse / TIMEFACTOR)) * WINDFACTOR
```

## Rain Amount

Another example for the extended pulse counter is the measurement of rain and rain downpour. While the ESP32 is in Deep Sleep, the ULP counts the pulses of a Rain Gauge.
The rain amount can be calculated easily, because each pulse of the Rain Gauge has a defined amount. On top of that, the shortest pulse can be used to calculate the
highest rain amount, which I defined as the rain downpour value.

```
RAINFACTOR = 0.2794  // 0.2794 mm per pulse defined by the Rain Gauge
TIMEFACTOR = 1000000 // factor between seconds and microseconds
```

### Pulse Count Usage
```C
float rain = pulses * RAINFACTOR
```

### Shortest Pulse Usage
```C
float rainDownpour = (timeSleep / (shortestPulse / TIMEFACTOR)) * RAINFACTOR
```
