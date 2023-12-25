.. _peripheral_ancsc:

Bluetooth: Peripheral ANCS client
#################################


The Peripheral ANCS client sample demonstrates how to get the notification data from iOS.

Requirements
************

The sample supports the following development kits:

.. A Apollo Blue EVB (for example, apollo4p_blue_kxr_evb).
.. A device running an ANCS Server to connect with (for example, an iPhone which runs iOS).


The following table shows the format of a notification that you can send to the example application:

   +------------------+---------------+--------------------------+
   | Field            | Example value | Interpretation           |
   +==================+===============+==========================+
   | Event ID         | 0             | Notification added       |
   +------------------+---------------+--------------------------+
   | Flags            | 18            | Positive/negative action |
   +------------------+---------------+--------------------------+
   | Category         | 06            | Email                    |
   +------------------+---------------+--------------------------+
   | Category count   | 02            |                          |
   +------------------+---------------+--------------------------+
   | Notification UID | 01 02 03 04   | 67305985 (0x4030201)     |
   +------------------+---------------+--------------------------+

#. In the Bluetooth Low Energy app, set the value of **Apple Notification Source** to ``00 18 06 02 01 02 03 04`` and click :guilabel:`Update`.
#. Verify that the UART data is received as follows::

      Notification
      Event:       Added
      Category ID: Email
      Category Cnt:2
      UID:         67305985
      Flags:
       Positive Action
       Negative Action

Perform a notification action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The received notification has two flags: Positive Action and Negative Action.
This means that you can perform these two actions on the notification.

The following table shows the format of the message that the application must send back to perform a notification action:

   +------------------+---------------+-----------------------------+
   | Field            | Example value | Interpretation              |
   +==================+===============+=============================+
   | Command ID       | 2             | Perform notification action |
   +------------------+---------------+-----------------------------+
   | Notification UID | 01 02 03 04   | 67305985 (0x4030201)        |
   +------------------+---------------+-----------------------------+
   | Action           | 00            | Positive                    |
   |                  |               |                             |
   |                  | 01            | Negative                    |
   +------------------+---------------+-----------------------------+

* Press **Button 3** to perform a positive action and verify that the **Apple Control Point** value is updated to ``02 01 02 03 04 00``.
* You can also press **Button 4** and observe that the server receives a negative action ``02 01 02 03 04 01``.

Retrieve notification attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following table shows the relevant part of a request to retrieve notification attributes:

   +------------------+---------------+-----------------------------+
   | Field            | Example value | Interpretation              |
   +==================+===============+=============================+
   | Command ID       | 0             | Get notification attributes |
   +------------------+---------------+-----------------------------+
   | Notification UID | 01 02 03 04   | 67305985 (0x4030201)        |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 00            | App identifier              |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 01            | Title                       |
   +------------------+---------------+-----------------------------+
   | Length           | 20 00         | 0x0020                      |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 03            | Message                     |
   +------------------+---------------+-----------------------------+
   | Length           | 20 00         | 0x0020                      |
   +------------------+---------------+-----------------------------+

Note that the example application will request all existing attribute types, not only a subset.

The following table shows the format of a response that contains some of the requested notification attributes:

   +------------------+---------------+-----------------------------+
   | Field            | Example value | Interpretation              |
   +==================+===============+=============================+
   | Command ID       | 0             | Get notification attributes |
   +------------------+---------------+-----------------------------+
   | Notification UID | 01 02 03 04   | 67305985 (0x4030201)        |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 01            | Title                       |
   +------------------+---------------+-----------------------------+
   | Length           | 03 00         | 0x0003                      |
   +------------------+---------------+-----------------------------+
   | Data             | 6E 52 46      | "nRF"                       |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 03            | Message                     |
   +------------------+---------------+-----------------------------+
   | Length           | 02 00         | 0x0002                      |
   +------------------+---------------+-----------------------------+
   | Data             | 35 32         | "52"                        |
   +------------------+---------------+-----------------------------+
   | Attribute ID     | 00            | App identifier              |
   +------------------+---------------+-----------------------------+
   | Length           | 03 00         | 0x0003                      |
   +------------------+---------------+-----------------------------+
   | Data             | 63 6F 6D      | "com"                       |
   +------------------+---------------+-----------------------------+

#. Press **Button 1** to request notification attributes for the iOS notification that was received.
#. In the Bluetooth Low Energy app, verify that the **Apple Control Point** is updated to ``00 01 02 03 04 00 01 20 00 02 20 00 03 20 00 04 05 06 07``.
#. Respond to the request by sending two notification attributes: the title and the message.
   Set the **Apple Data Source** value in the Server to ``00 01 02 03 04 01 03 00 6E 52 46 03 02 00 35 32``.
#. The application will print the received data on UART.
   Verify that the UART output is as follows::

      Title: nRF
      Message: 52

#. Update the **Apple Data Source** value again with the app identifier ``00 03 00 63 6F 6D``.
#. Verify that the notification is received and the UART output is as follows::

      App Identifier: com

Retrieve app attributes
^^^^^^^^^^^^^^^^^^^^^^^

With the app identifier, you can request attributes of the app that sent the notification.

The following table shows the format of a request to retrieve app attributes:

   +------------------+---------------+--------------------+
   | Field            | Example value | Interpretation     |
   +==================+===============+====================+
   | Command ID       | 1             | Get app attributes |
   +------------------+---------------+--------------------+
   | App identifier   | 6D 6F 63 00   | "com" + '\0'       |
   +------------------+---------------+--------------------+
   | Attribute ID     | 0             | Display name       |
   +------------------+---------------+--------------------+

The following table shows the format of a response that contains the requested app attributes:

   +------------------+---------------+--------------------+
   | Field            | Example value | Interpretation     |
   +==================+===============+====================+
   | Command ID       | 1             | Get app attributes |
   +------------------+---------------+--------------------+
   | App identifier   | 6D 6F 63 00   | "com" + '\0'       |
   +------------------+---------------+--------------------+
   | Attribute ID     | 0             | Display name       |
   +------------------+---------------+--------------------+
   | Length           | 04 00         | 0x0004             |
   +------------------+---------------+--------------------+
   | Data             | 4D 61 69 6C   | "Mail"             |
   +------------------+---------------+--------------------+

#. Press **Button 2** to request app attributes for the app with the app identifier "com" (the last received app identifier).
#. In the Bluetooth Low Energy app, verify that the **Apple Control Point** is updated to ``01 63 6F 6D 00 00``.
#. Respond to the request by sending the app attribute.
   Set the **Apple Data Source** value in the Server to ``01 63 6F 6D 00 00 04 00 4D 61 69 6C``.
#. The application will print the received data on UART. Verify that the UART output is as follows::

      Display Name: Mail
