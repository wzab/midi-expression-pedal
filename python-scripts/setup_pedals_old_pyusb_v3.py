#!/usr/bin/env python
#
import usb
busses = usb.busses()

for bus in busses:
  devices = bus.devices
  for dev in devices:
    if dev.idVendor==0x16c0 and dev.idProduct==0x05e4:
      # this may be our device...
      # open it
      dh = dev.open()
      if dh.getString(dev.iProduct,1000)=='V-USB-MIDI-PEDAL+SWITCH' and\
         dev.deviceVersion == '00.03':
         # This is one of our devices
         ser_num = dh.getString(dev.iSerialNumber,1000)
         # Configure devices depending ontheir serial number
         if ser_num == '0101':
           # Device found, configure it properly
           # To know how to build appropriate USB midi event packets
           # see http://www.usb.org/developers/devclass_docs/midi10.pdf 
           # pages 16 and next
           # see also http://www.midi.org/techspecs/midimessages.php
           #
           # Our control message transfer must have the following form
           # 1st argument - bmRequestType - 0x40
           # 2nd argument - bRequest 
           #   - 0x8n to configure packet sent when n-th switch gets pressed
           #   - 0xcn to configure packet sent when n-th switch gest released
           # 3rd argument - wIndex - first 2 bytes of packet 0xB0B1 (B0-first byte, B1-second byte)
           # 4th argument - wValue - next 2 bytes of packet 0xB2B3 (B2-third byte, B3-fourth byte)
           # The example below configures switches 4 and 5 to send program changes 1 and 2 when pressed
           dh.controlMsg(0x40,0x84,"",0x0cc0,0x0100)
           dh.controlMsg(0x40,0x85,"",0x0cc0,0x0700)
           # Two transfers below configure switch 3 to emulate the control, which can be used
           # e.g. to control "looper" in guitarix (the looper starts recording when you switch the button
           # and stops recording, when you release it)
           dh.controlMsg(0x40,0x83,"",0x0bb0,0x507f)
           dh.controlMsg(0x40,0xc3,"",0x0bb0,0x5000)
         if ser_num == '0102':
           # This device may be configured in a different way 
           # switches 3,4 and 5 select programs 0,1 and 2
           dh.controlMsg(0x40,0x83,"",0x0cc0,0x0000)
           dh.controlMsg(0x40,0x84,"",0x0cc0,0x0100)
           dh.controlMsg(0x40,0x85,"",0x0cc0,0x0200)
           # Change "polarity" of the pedal and assign it to the modulation (0x01) controller
           dh.controlMsg(0x40,0xbf,"",0x0bb0,0x01ff)



