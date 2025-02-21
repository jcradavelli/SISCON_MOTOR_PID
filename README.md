


| Supported Targets | ESP32 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |
|                   | [![.github/workflows/main.yml](https://github.com/jcradavelli/SISCON_MOTOR_PID/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/jcradavelli/SISCON_MOTOR_PID/actions/workflows/main.yml) | não testado | não testado | não testado | não testado |



# Rotary Encoder

Este projeto utiliza o periférico PCNT (pulse counter) para fazer o tratamento dos sinais de encoder. A baixo segue a descrição de funcionamento fornecida pela expressif:


The PCNT peripheral is designed to count the number of rising and/or falling edges of an input signal. Each PCNT unit has two channels, which makes it possible to extract more information from two input signals than only one signal.
This example shows how to make use of the HW features to decode the differential signals generated from a common rotary encoder -- [EC11](https://tech.alpsalpine.com/prod/e/html/encoder/incremental/ec11/ec11_list.html).

The signals a rotary encoder produces (and what can be handled by this example) are based on a 2-bit gray code available on 2 digital data signal lines. The typical encoders use 3 output pins: 2 for the signals and one for the common signal usually GND.

Typical signals:

```
A      +-----+     +-----+     +-----+
             |     |     |     |
             |     |     |     |
             +-----+     +-----+
B         +-----+     +-----+     +-----+
                |     |     |     |
                |     |     |     |
                +-----+     +-----+

 +--------------------------------------->
                CW direction
```

## Informações gerais

### Hardware Required

Em constrção 

Connection :

```
      +--------+              +---------------------------------+
      |        |              |                                 |
      |      A +--------------+ GPIO_A (internal pull up)       |
      |        |              |                                 |
+-------+      |              |                                 |
|     | |  GND +--------------+ GND                             |
+-------+      |              |                                 |
      |        |              |                                 |
      |      B +--------------+ GPIO_B (internal pull up)       |
      |        |              |                                 |
      +--------+              +---------------------------------+
```



### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.


