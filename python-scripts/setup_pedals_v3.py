#!/usr/bin/python
import usb.core
import usb.util
def getStringDescriptor(device, index):
   """
   """
   response = device.ctrl_transfer(usb.util.ENDPOINT_IN,
                                  usb.legacy.REQ_GET_DESCRIPTOR,
                                  (usb.util.DESC_TYPE_STRING << 8) | index,
                                  0, # language id
                                  255) # length
   return response[2:].tostring().decode('utf-16')

def is_our_MIDI_pedal(dev):
   name=u'V-USB-MIDI-PEDAL+SWITCH'
   version=0x0003
   return (dev.bcdDevice == version and \
           getStringDescriptor(dev,dev.iProduct) == name)

# We assume, that our device is the first one with 
# the VUSB shared idVendor and idProdect, 
# and with proper name and version
devs = usb.core.find(find_all=True, idVendor=0x16c0, idProduct=0x05e4, \
		custom_match=is_our_MIDI_pedal)
for dev in devs:
  # Read the serial number - to support multiple devices
  # we allow to program different devices in different way
  # Each device should have unique serial number
  ser_num = getStringDescriptor(dev, dev.iSerialNumber)
  if ser_num == u'0101':
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
    dev.ctrl_transfer(0x40,0x84,0x0cc0,0x0100)
    dev.ctrl_transfer(0x40,0x85,0x0cc0,0x0200)
    # Two transfers below configure switch 3 to emulate the control, which can be used
    # e.g. to control "looper" in guitarix (the looper starts recording when you switch the button
    # and stops recording, when you release it)
    dev.ctrl_transfer(0x40,0x83,0x0bb0,0x507f)
    dev.ctrl_transfer(0x40,0xc3,0x0bb0,0x5000)
  if ser_num == u'0102':
    # This device may be configured in a different way 
    # switches 3,4 and 5 select programs 0,1 and 2
    dev.ctrl_transfer(0x40,0x83,0x0cc0,0x0000)
    dev.ctrl_transfer(0x40,0x84,0x0cc0,0x0100)
    dev.ctrl_transfer(0x40,0x85,0x0cc0,0x0200)
    # Change "polarity" of the pedal and assign it to the modulation (0x01) controller
    dev.ctrl_transfer(0x40,0xbf,0x0bb0,0x01ff)
if len(devs)==0:
  print "Device not found!"

