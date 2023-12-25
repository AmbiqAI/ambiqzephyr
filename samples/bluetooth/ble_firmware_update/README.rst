.. _ble_firmware_update:

ble_firmware_update
###################

Overview
********

A sample that mainly be used to update the BLE controller firmware
of Ambiq Apollo4x soc with Bluetooth Low Engegy supported. The user
can replace the header file in include directory and uncomment the
corresponding macro in main.c for the update.


Building and Running
********************

This application can be built and executed as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/ble_firmware_update
   :board: apollo4p_blue_kxr_evb

Sample Output
=============

.. code-block:: console

   le_firmware_update: BLE Firmware Update Application
   bt_controller: BLE Controller Info:
   bt_controller: 	FW Ver:      1.21.2.0
   bt_controller: 	Chip ID0:    0x92492492
   bt_controller: 	Chip ID1:    0x4190d413
   bt_controller: No new image to upgrade
   bt_controller: BLE Controller FW Auth Passed, Continue with FW
   bt_apollox_driver: BT controller initialized
   bt_hci_core: Identity: 22:89:67:45:23:22 (public)
   bt_hci_core: HCI: version 5.1 (0x0a) revision 0x0115, manufacturer 0x09ac
   bt_hci_core: LMP: version 5.1 (0x0a) subver 0x0115
   ble_firmware_update: Reset BLE controller to do a forcing upgrade
   bt_controller: BLE Controller Info:
   bt_controller: 	FW Ver:      1.21.2.0
   bt_controller: 	Chip ID0:    0x92492492
   bt_controller: 	Chip ID1:    0x4190d413
   bt_controller: BLE Controller Requires FW
   bt_controller: BLE controller upgrade in progress, wait...
   bt_controller: Update Done
   ble_firmware_update: BLE Firmware Update Application Done!
