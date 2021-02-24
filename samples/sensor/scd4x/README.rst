.. _scd4x:

Sensirion SCD4x CO2 Sensor
###################################

Overview
********

This sample application periodically reads CO2, ambient temperature and humidity
data from the first available device that implements SENSOR_CHAN_CO2,
SENSOR_CHAN_PRESS, and SENSOR_CHAN_HUMIDITY.
This sample checks the sensor in polling mode (without interrupt trigger).

Optionally calibration procedures can be executed at the beginning.

The sensor provides 3 measurement modes that can be changed at runtime.

* High performance mode for 5s sample intervals
* Low power mode for 30s sample intervals
* Single shot On-demand mode(SCD41 only)


The sensor parameters can be reset to factory defaults during init, by enabling
``CONFIG_SCD4X_FACTORY_RESET_AT_INIT``.
The Sensor optionally executes a selftest during driver initialization in case
``CONFIG_SCD4X_SELFTEST_AT_INIT`` is enabled.



Building and Running
********************

This sample application uses an SCD41 sensor connected to a board that has
defined arduino i2c pins.
For evaluation purposes you can use a SEK-SCD41 evaluation Kit.
Connect the sensor pins according to the connection diagram given in the
`sdc4x datasheet`_ at page 5 figure 1.


.. zephyr-app-commands::
   :zephyr-app: samples/sensor/scd4x
   :board: nrf52840dk_nrf52840
   :goals: flash
   :compact:

Sample Output
=============
To check output of this sample , any serial console program can be used.
This example uses ``picocom`` on the serial port ``/dev/ttyACM0``:

.. code-block:: console

        $ sudo picocom -D /dev/ttyUSB0

.. code-block:: console

        CO2: 440; temp: 20.571027; humidity: 61.014648
        CO2: 441; temp: 20.570269; humidity: 61.012695
        CO2: 440; temp: 20.570695; humidity: 61.002929

.. _scd4x datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD4x_Datasheet.pdf
