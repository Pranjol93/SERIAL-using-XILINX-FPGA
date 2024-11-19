Title: Design and Implementation of a Serial IP Core and Linux Device Drivers

This project focuses on developing a programmable Serial IP module integrated with an AXI4-lite interface. The module provides a UART-based communication system capable of supporting configurable baud rates (75-250,000), data word sizes (5-8 bits), optional parity, and one or two stop bits.

Key Features:

Hardware: TX/RX communication without handshaking, programmable baud rate recovery clock, and FIFO for TX/RX (9-bit wide, 16-depth).
Software: Linux kernel module with virtual memory (/dev/mem) and sysfs-based controls (/sys/class/uart).
Interrupt handling for efficient operation.
Verification using simulation tools and hardware testing in a supervised lab environment.
This project showcases skills in hardware-software co-design, Verilog programming, and Linux kernel development.
