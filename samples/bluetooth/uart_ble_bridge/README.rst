.. _bluetooth_uart_ble_bridge:

Bluetooth: UART BLE Bridge
##########################

Overview
********

This sample works as the bridge to passthrough the HCI commands sent from
the host (e.g. test machine, serial tool running in PC or others) to
the Apollox BLE controller via SPI, and passthrough the received response
sent from the Apollox BLE controller to the host via UART.
This sample is always used for DTM testing purpose.

Requirements
************

* A apollo4p_blue_kxr_evb or other Ambiq Apollox Blue EVB.

Default UART settings
*********************

By default the controller builds use the following settings:

* Baudrate: 1Mbit/s
* 8 bits, no parity, 1 stop bit
* Hardware Flow Control (RTS/CTS) enabled

Building and Running
********************

This application can be built and executed as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/uart_ble_bridge
   :board: apollo4p_blue_kxr_evb

The uart0 will be used to receive the HCI commands from host, while
uart1 is used for the console output.

The uart tx/rx from host aspect will look like:
 TX: 01 03 0C 00
 RX: 04 0E 04 05 03 0C 00
