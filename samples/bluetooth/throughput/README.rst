.. _throughput:

Bluetooth: Throughput
#####################

Overview
********

A sample to demostrate the Ambiq BLE throughput service. The APP
can configure the MTU PHY and test direction in the home page after
connecting the board, and show the throughput statistics.

Requirements
************

* A apollo4p_blue_kxr_evb
* A Android smartphone installed with ambiq_ble_tester APP

Building and Running
********************

This application can be built and executed as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/throughput
   :board: apollo4p_blue_kxr_evb
