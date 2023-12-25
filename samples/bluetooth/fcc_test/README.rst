.. _ble_fcc_test:

Bluetooth: FCC Test
###################

Overview
********

This sample is used to perform the FCC testing for Ambiq Apollox Blue
series soc. If there are two buttons on the EVB, the user can press
button0 to switch phy (LE 1M or LE 2M) and press button1 to switch test
case (e.g. tx continuous wave, tx continuous modulate, rx test on channel
2402MHz, 2440MHz or 2480MHz). The testing should be worked with RF test
machine (e.g. CWM500) to capture the RF signal.

Requirements
************

* A Ambiq Apollox Blue EVB with Bluetooth LE support, e.g. apollo4p_blue_kxr_evb
* A RF test machine, e.g. CWM500
* A RF test antenna to connect the EVB with the RF test machine

Building and Running
********************

This application can be built and executed as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/fcc_test
   :board: apollo4p_blue_kxr_evb
