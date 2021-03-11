.. _shtcx_sample:

Sensirion SHTCx Sample
######################

Description
***********

This sample application periodically measures Temperature and Humidity
using the SHTCx sensor driver. The result is written to the console.

Requirements
************

This sample needs a compatible sensor and an enabled node with
(``compatible = sensirion,shtcx``).
The driver is currently only tested using a SHTC3 chip,
but should also be compatible with SHTC1 and SHTW1 chips.
The chip is identified using the dt property ``chip`` , which selects shtc3 in
the samples app.overlay file and must be overwritten to use it with shtc1.

Example Breakout Boards:

* Sparkfun SEN-16467: Humidity Sensor Breakout board - SHTC3
* Adafruit 4636: Adafruit Sensirion SHTC3 Temperature & Humidity Sensor
* MIKROE 3331: TEMP&HUM 9 CLICK


Building and Running
********************

Connect the sensor pins according to the connection diagram given in the
`shtc1 datasheet`_ at page 6 figure 6, `shtc3 datasheet`_ at page 6 figure 6,
or as described for you breakout board.

The SHTC3 sensor operates at 1.62V to 3.6V and uses I2C to communicate
with the board.

This sample builds one application for the SHTC3 sensor.
Build/Flash Steps:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/shtcx
   :board: nucleo_l496zg
   :goals: build flash
   :compact:

Sample Output
*************
.. code-block:: console

    Found device "SHTC3", getting sensor data
    Temp = 23.305053 C, RH = 49.273681 %
    Temp = 23.315124 C, RH = 49.475097 %
    ...


References
**********

.. target-notes::

.. _shtc3 datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHTC3_Datasheet.pdf
.. _shtc1 datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHTC1_Datasheet.pdf
