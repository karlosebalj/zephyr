.. _mcp2518fd_click_shield:

Microchip MCP2518FD Click shield (CAN FD)
#########################################

Overview
--------

MCP2518FD Click shield has a MCP2518FD CAN FD controller via an SPI
interface and a high-speed ATA6563 CAN transceiver.

More information about the shield can be found at
`MCP2518FD click`_.

Requirements
************

The shield uses a mikroBUS interface. The target board must define
a mikrobus_spi node label (see :ref:`shields` for more details).

Programming
***********

Set ``-DSHIELD=mcp2518fd_click`` when you invoke ``west build``,
for example:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/counter
   :board: lpcxpresso55s28
   :shield: mcp2518fd_click
   :goals: build flash

References
**********

.. target-notes::

.. _MCP2518FD click:
   https://www.mikroe.com/mcp2518fd-click
