.. _arduino_uno_click:

Arduino UNO click shield
########################

Overview
********

The Arduino UNO click is an extension to the Arduino UNO R3 headers.
It's a simple shield that converts Arduino UNO R3 headers to two mikroBUS
host sockets that allow you to connect many other click shields to your
board.
In other words, he Arduino UNO click will generally be used by other
shields using the mikroBUS interface.

Two mikroBUS headers are exposed by the overlay: mikrobus_headers_1 and
mikrobus_headers_2, each corresponding to a socket on the Arduino UNO
click shield.

More information about the shield can be found at
`Arduino UNO click shield website`_.

Requirements
************

This shield can only be used with a board which provides a configuration
for Arduino R3 connector.

Since the Arduino UNO click has two sockets, the user must create a node
alias depeding on which of the sockets is used:

mikrobus_header: &mikrobus_header_1 {}; or
mikrobus_header: &mikrobus_header_2 {};

The board must also define node aliases for arduino Serial,
SPI and I2C interfaces (see :ref:`shields` for more details).

If a node alias for one of the interfaces does not exist and it is not used
by the connecting shield, a dummy node alias can be created by the board.
An alternative is to simply delete the node alias from the
arduino_uno_click.overlay file. For example if a connecting shield only uses
the mikrobus_spi node alias, then mikrobus_serial and mikrobus_i2c can be deleted.

Programming
***********

Include ``-DSHIELD=arduino_uno_click`` when you invoke ``west build`` with
other mikroBUS shields. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/counter
   :board: xmc47_relax_kit
   :shield: "arduino_uno_click mcp2518fd_click"
   :goals: build

References
**********

.. target-notes::

.. _Arduino UNO click shield website:
   https://www.mikroe.com/arduino-uno-click-shield
