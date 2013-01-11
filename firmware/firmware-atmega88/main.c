/* Name: main.c
 * Project: V-USB MIDI expression pedal with capacitive sensor
 * Author: Wojciech M. Zabolotny (wzab@ise.pw.edu.pl)
 * Creation Date: 2010-02-27
 * Copyright: (c) 2010 by Wojciech M. Zabolotny
 * License: GPL.
 *
 * Significantly based on the project:
 * Project: V-USB MIDI device on Low-Speed USB
 * Author: Martin Homuth-Rosemann
 * Creation Date: 2008-03-11
 * Copyright: (c) 2008 by Martin Homuth-Rosemann.
 * License: GPL.
 *
 * 
 */

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "usbdrv.h"
#include "oddebug.h"

#if DEBUG_LEVEL > 0
#	warning "Never compile production devices with debugging enabled"
#endif

#define FILTERING_LEVEL 3

//Switch constants
#define NUM_OF_SWITCHES (10)
#define SWITCH_THRESHOLD (10)
#define KEY_THRESHOLD (60)
static unsigned char key_state = 0;
static unsigned char switch_state[NUM_OF_SWITCHES] = {[0 ... NUM_OF_SWITCHES-1]=0};
static unsigned char switch_msg_on[NUM_OF_SWITCHES][4] = {[0 ... NUM_OF_SWITCHES-1]={0x0c,0xc0,0x0,0}};
static unsigned char switch_msg_off[NUM_OF_SWITCHES][4] = {[0 ... NUM_OF_SWITCHES-1]={0x0,0x0,0x0,0}};
static unsigned char pedal_msg[4] = {0x0b, 0xb0, 0x70, 0x00};
enum t_switch_sent
  {
    ON_SENT,
    OFF_SENT,
  };
enum t_switch_sent switch_sent[NUM_OF_SWITCHES] = {[0 ... NUM_OF_SWITCHES-1]=OFF_SENT};

// This descriptor is based on http://www.usb.org/developers/devclass_docs/midi10.pdf
// 
// Appendix B. Example: Simple MIDI Adapter (Informative)
// B.1 Device Descriptor
//
static PROGMEM char deviceDescrMIDI[] = {	/* USB device descriptor */
  18,			/* sizeof(usbDescriptorDevice): length of descriptor in bytes */
  USBDESCR_DEVICE,	/* descriptor type */
  0x10, 0x01,		/* USB version supported */
  0,			/* device class: defined at interface level */
  0,			/* subclass */
  0,			/* protocol */
  8,			/* max packet size */
  USB_CFG_VENDOR_ID,	/* 2 bytes */
  USB_CFG_DEVICE_ID,	/* 2 bytes */
  USB_CFG_DEVICE_VERSION,	/* 2 bytes */
  1,			/* manufacturer string index */
  2,			/* product string index */
  3,			/* serial number string index */
  1,			/* number of configurations */
};

// B.2 Configuration Descriptor
static PROGMEM char configDescrMIDI[] = {	/* USB configuration descriptor */
  9,			/* sizeof(usbDescrConfig): length of descriptor in bytes */
  USBDESCR_CONFIG,	/* descriptor type */
  101, 0,			/* total length of data returned (including inlined descriptors) */
  2,			/* number of interfaces in this configuration */
  1,			/* index of this configuration */
  0,			/* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
  USBATTR_SELFPOWER,	/* attributes */
#else
  USBATTR_BUSPOWER,	/* attributes */
#endif
  USB_CFG_MAX_BUS_POWER / 2,	/* max USB current in 2mA units */

  // B.3 AudioControl Interface Descriptors
  // The AudioControl interface describes the device structure (audio function topology) 
  // and is used to manipulate the Audio Controls. This device has no audio function 
  // incorporated. However, the AudioControl interface is mandatory and therefore both 
  // the standard AC interface descriptor and the classspecific AC interface descriptor 
  // must be present. The class-specific AC interface descriptor only contains the header 
  // descriptor.

  // B.3.1 Standard AC Interface Descriptor
  // The AudioControl interface has no dedicated endpoints associated with it. It uses the 
  // default pipe (endpoint 0) for all communication purposes. Class-specific AudioControl 
  // Requests are sent using the default pipe. There is no Status Interrupt endpoint provided.
  /* AC interface descriptor follows inline: */
  9,			/* sizeof(usbDescrInterface): length of descriptor in bytes */
  USBDESCR_INTERFACE,	/* descriptor type */
  0,			/* index of this interface */
  0,			/* alternate setting for this interface */
  0,			/* endpoints excl 0: number of endpoint descriptors to follow */
  1,			/* */
  1,			/* */
  0,			/* */
  0,			/* string index for interface */

  // B.3.2 Class-specific AC Interface Descriptor
  // The Class-specific AC interface descriptor is always headed by a Header descriptor 
  // that contains general information about the AudioControl interface. It contains all 
  // the pointers needed to describe the Audio Interface Collection, associated with the 
  // described audio function. Only the Header descriptor is present in this device 
  // because it does not contain any audio functionality as such.
  /* AC Class-Specific descriptor */
  9,			/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
  36,			/* descriptor type */
  1,			/* header functional descriptor */
  0x0, 0x01,		/* bcdADC */
  9, 0,			/* wTotalLength */
  1,			/* */
  1,			/* */

  // B.4 MIDIStreaming Interface Descriptors

  // B.4.1 Standard MS Interface Descriptor
  /* interface descriptor follows inline: */
  9,			/* length of descriptor in bytes */
  USBDESCR_INTERFACE,	/* descriptor type */
  1,			/* index of this interface */
  0,			/* alternate setting for this interface */
  2,			/* endpoints excl 0: number of endpoint descriptors to follow */
  1,			/* AUDIO */
  3,			/* MS */
  0,			/* unused */
  0,			/* string index for interface */

  // B.4.2 Class-specific MS Interface Descriptor
  /* MS Class-Specific descriptor */
  7,			/* length of descriptor in bytes */
  36,			/* descriptor type */
  1,			/* header functional descriptor */
  0x0, 0x01,		/* bcdADC */
  65, 0,			/* wTotalLength */

  // B.4.3 MIDI IN Jack Descriptor
  6,			/* bLength */
  36,			/* descriptor type */
  2,			/* MIDI_IN_JACK desc subtype */
  1,			/* EMBEDDED bJackType */
  1,			/* bJackID */
  0,			/* iJack */

  6,			/* bLength */
  36,			/* descriptor type */
  2,			/* MIDI_IN_JACK desc subtype */
  2,			/* EXTERNAL bJackType */
  2,			/* bJackID */
  0,			/* iJack */

  //B.4.4 MIDI OUT Jack Descriptor
  9,			/* length of descriptor in bytes */
  36,			/* descriptor type */
  3,			/* MIDI_OUT_JACK descriptor */
  1,			/* EMBEDDED bJackType */
  3,			/* bJackID */
  1,			/* No of input pins */
  2,			/* BaSourceID */
  1,			/* BaSourcePin */
  0,			/* iJack */

  9,			/* bLength of descriptor in bytes */
  36,			/* bDescriptorType */
  3,			/* MIDI_OUT_JACK bDescriptorSubtype */
  2,			/* EXTERNAL bJackType */
  4,			/* bJackID */
  1,			/* bNrInputPins */
  1,			/* baSourceID (0) */
  1,			/* baSourcePin (0) */
  0,			/* iJack */


  // B.5 Bulk OUT Endpoint Descriptors

  //B.5.1 Standard Bulk OUT Endpoint Descriptor
  9,			/* bLenght */
  USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
  0x1,			/* bEndpointAddress OUT endpoint number 1 */
  3,			/* bmAttributes: 2:Bulk, 3:Interrupt endpoint */
  8, 0,			/* wMaxPacketSize */
  10,			/* bIntervall in ms */
  0,			/* bRefresh */
  0,			/* bSyncAddress */

  // B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
  5,			/* bLength of descriptor in bytes */
  37,			/* bDescriptorType */
  1,			/* bDescriptorSubtype */
  1,			/* bNumEmbMIDIJack  */
  1,			/* baAssocJackID (0) */


  //B.6 Bulk IN Endpoint Descriptors

  //B.6.1 Standard Bulk IN Endpoint Descriptor
  9,			/* bLenght */
  USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
  0x81,			/* bEndpointAddress IN endpoint number 1 */
  3,			/* bmAttributes: 2: Bulk, 3: Interrupt endpoint */
  8, 0,			/* wMaxPacketSize */
  10,			/* bIntervall in ms */
  0,			/* bRefresh */
  0,			/* bSyncAddress */

  // B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
  5,			/* bLength of descriptor in bytes */
  37,			/* bDescriptorType */
  1,			/* bDescriptorSubtype */
  1,			/* bNumEmbMIDIJack (0) */
  3,			/* baAssocJackID (0) */
};


uchar usbFunctionDescriptor(usbRequest_t * rq)
{

  if (rq->wValue.bytes[1] == USBDESCR_DEVICE) {
    usbMsgPtr = (uchar *) deviceDescrMIDI;
    return sizeof(deviceDescrMIDI);
  } else {		/* must be config descriptor */
    usbMsgPtr = (uchar *) configDescrMIDI;
    return sizeof(configDescrMIDI);
  }
}


static uchar sendEmptyFrame;


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

uchar usbFunctionSetup(uchar data[8])
{
  usbRequest_t *rq = (void *) data;


  if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {	/* class request type */

    /*  Prepare bulk-in endpoint to respond to early termination   */
    if ((rq->bmRequestType & USBRQ_DIR_MASK) ==
	USBRQ_DIR_HOST_TO_DEVICE)
      sendEmptyFrame = 1;
  } else if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR) {	/* vendor request type */
    if ((rq->bmRequestType & USBRQ_DIR_MASK) ==
	USBRQ_DIR_HOST_TO_DEVICE) {
      //This is our message, used to configure the device
      //if bits 6 and 7 are set to "10", then this request sets the ON message for the switch
      //or the pedal (if the 0x3f switch is selected: 0x80+0x3f=0xbf)
      if(rq->bRequest == 0xbf) {
	//Reprogramming of pedal
	pedal_msg[0]=rq->wValue.bytes[1];
	pedal_msg[1]=rq->wValue.bytes[0];
	pedal_msg[2]=rq->wIndex.bytes[1];
	pedal_msg[3]=rq->wIndex.bytes[0];
	return 0;
      } else if((rq->bRequest & 0xc0) == 0x80) {
	if((rq->bRequest & 0x3f) < NUM_OF_SWITCHES) {
	  unsigned char i = rq->bRequest & 0x3f;
	  switch_msg_on[i][0] = rq->wValue.bytes[1];
	  switch_msg_on[i][1] = rq->wValue.bytes[0];
	  switch_msg_on[i][2] = rq->wIndex.bytes[1];
	  switch_msg_on[i][3] = rq->wIndex.bytes[0];
	  return 0;
	}
      } else
	//if bits 6 and 7 are set to "11", then this request sets the OFF message for the switch
	if((rq->bRequest & 0xc0) == 0xc0) {
	  if((rq->bRequest & 0x3f) < NUM_OF_SWITCHES) {
	    unsigned char i = rq->bRequest & 0x3f;
	    switch_msg_off[i][0] = rq->wValue.bytes[1];
	    switch_msg_off[i][1] = rq->wValue.bytes[0];
	    switch_msg_off[i][2] = rq->wIndex.bytes[1];
	    switch_msg_off[i][3] = rq->wIndex.bytes[0];
	    return 0;
	  }
	
	}
    }
  }
  return 0x0; //Ignore unknown!
}


/*---------------------------------------------------------------------------*/
/* usbFunctionRead                                                           */
/*---------------------------------------------------------------------------*/

uchar usbFunctionRead(uchar * data, uchar len)
{
  // DEBUG LED

  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;

  return 7;
}


/*---------------------------------------------------------------------------*/
/* usbFunctionWrite                                                          */
/*---------------------------------------------------------------------------*/

uchar usbFunctionWrite(uchar * data, uchar len)
{

  return 1;
}


/*---------------------------------------------------------------------------*/
/* usbFunctionWriteOut                                                       */
/*                                                                           */
/* this Function is called if a MIDI Out message (from PC) arrives.          */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usbFunctionWriteOut(uchar * data, uchar len)
{

}



/*---------------------------------------------------------------------------*/
/* hardwareInit                                                              */
/*---------------------------------------------------------------------------*/

static void hardwareInit(void)
{
  uchar i, j;

  /* activate pull-ups except on USB lines */
  USB_CFG_IOPORT =
    (uchar) ~ ((1 << USB_CFG_DMINUS_BIT) |
	       (1 << USB_CFG_DPLUS_BIT));
  /* all pins input except USB (-> USB reset) */
#ifdef USB_CFG_PULLUP_IOPORT	/* use usbDeviceConnect()/usbDeviceDisconnect() if available */
  USBDDR = 0;		/* we do RESET by deactivating pullup */
  usbDeviceDisconnect();
#else
  USBDDR = (1 << USB_CFG_DMINUS_BIT) | (1 << USB_CFG_DPLUS_BIT);
#endif

  j = 0;
  while (--j) {		/* USB Reset by device only required on Watchdog Reset */
    i = 0;
    while (--i);	/* delay >10ms for USB reset */
  }
#ifdef USB_CFG_PULLUP_IOPORT
  usbDeviceConnect();
#else
  USBDDR = 0;		/*  remove USB reset condition */
#endif

  //PORTA = 0xff;   /* activate all pull-ups */
  //DDRA = 0;       /* all pins input */
  ACSR=(0 << ACIC); /* Input Capture taken from ICP1 */
  // keys/switches setup
  // PORTD.5 is used to control the LED
  DDRD |= 0x20;
  // The switch is connected to the pin 4 of PORTD
  // We need pullups everywhere but not on USB pins
  PORTD |= 0xe3;

  PORTB = 0xfe;		/* enable all pull-ups except for PB0*/
  DDRB = 0x2;		/* PB1/OC1A as output */

  PORTC = 0xff;		/* all pullups on */
  DDRC = 0x00;		/* all pins input */
  //Setup TIMER0 (used for LED blinking)
  TCCR0A = 0;
  TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00) ; /* 256 prescaler - 47 kHz */
  //We will generate interrupts with frequency ca. 180Hz
  //increased comparing to previous firmwares to assure
  //better responsiveness
  //Setup TIMER1
  TCCR1A = (0 << COM1A1) | (1 << COM1A0) | /* Toggle on overflow */
    (0 << COM1B1) | (1 << COM1B0) | 
    (0 << WGM10 ) | (0 << WGM11) | /* Normal mode */
    0;
  TCCR1B = (0 << CS12) | (0 << CS11) | (1 << CS10) |
    (0 << WGM12) | (0 << WGM13) | /* Normal mode */
    (0 << ICES1) | /* Input capture on falling edge */
    0;
  //Finally we set TIMSK, so it can generate interrupts on
  // Input capture 1 (capacitance sensor)
  // Counter 0 overflow (led blinking)
  TIMSK1 = (1 << ICIE1);
  TIMSK0 = (1 << TOIE0); 
}

unsigned long value_icr = 0;
uchar new_icr = 0;
uchar flasher = 0b11001010;

ISR(TIMER1_CAPT_vect)
{
  sei();
  //Catch the time
#if FILTERING_LEVEL==0
  value_icr = ICR1; /* Without filter */
#else
  //Filter the value
  value_icr = value_icr - (value_icr >> FILTERING_LEVEL);
  value_icr += ICR1;
#endif
  //Mark, that the new value is received
  new_icr = 1;
}

unsigned char switch_released(unsigned char i)
{
  switch(i){
  case 0:
    return PINC & (1<<0);
  case 1:
    return PINC & (1<<1);
  case 2:
    return PINC & (1<<2);
  case 3:
    return PINC & (1<<3);
  case 4:
    return PINC & (1<<4);
  case 5:
    return PINC & (1<<5);
  case 6:
    return PIND & (1<<0);
  case 7:
    return PIND & (1<<1);
  case 8:
    return PIND & (1<<7);
  case 9:
    return PINB & (1<<2);
  }
  return 0;
}

ISR(TIMER0_OVF_vect)
{
  sei();
  static unsigned char tmr0_cnt = 0;
  register char bit_sel;
  register char i;
  //First - service the LED
  //Just increase the counter,
  tmr0_cnt ++;
  //Set the LED according to the proper bits from counter
  bit_sel = 1 << (0x7 & (tmr0_cnt >> 5));
  if (flasher & bit_sel)
    PORTD |= (1 << 5);
  else
    PORTD &= ~(1 << 5);
  //Now let's deal with the control switch
  //Deglitching routines for callibration key
  if(PIND & 0x40) {
    //key not pressed
    if (key_state > (255-KEY_THRESHOLD)) {
      //key was pressed and now is released
      key_state --;     
    } 
    else {
      //key was released long enough
      key_state = 0;
    } 
  } else {
    //key is pressed
    if (key_state < KEY_THRESHOLD) {
      //key was released but now is pressed
      key_state ++;
    }
    else {
      //key was pressed long enough
      key_state = 255;
    }
  }
  //Deglitching routines for switches
  for(i=0;i<NUM_OF_SWITCHES;i++) { 
    if(switch_released(i)) {
      //key not pressed
      if (switch_state[i] > (255-SWITCH_THRESHOLD)) {
	//key was pressed and now is released
	switch_state[i] --;     
      } 
      else {
	//key was released long enough
	switch_state[i] = 0;
      } 
    } else {
      //key is pressed
      if (switch_state[i] < SWITCH_THRESHOLD) {
	//key was released but now is pressed
	switch_state[i] ++;
      }
      else {
	//key was pressed long enough
	switch_state[i] = 255;
      }
    }
  }
}

int sensor(void)
{
  if (new_icr==0) return -1;
  new_icr = 0;
  return value_icr>>FILTERING_LEVEL; 
}


enum t_ctrl_state   /* Defines an enumeration type    */
  {
    ct_START,    
    ct_PRESSED_1,
    ct_GET_MAX,
    ct_WAIT_PRESS_1,
    ct_PRESSED_2,
    ct_GET_MIN,  
    ct_WORK,
  };

#define VAL_NUMBER 256

int main(void)
{
  int sensorOld = -1;
  uchar key;
  enum t_ctrl_state ctrl_state = ct_START;
  uchar midiMsg[8];
  int val, val_cnt=0;
  int val_tresh=0;
  long max_val=0, min_val=0, tmp_val=0;

 
  wdt_enable(WDTO_1S);
  hardwareInit();
  {
    //Initialize the "switch on" messages 
    //to send the appropriate program change number
    register char i;
    for (i=0;i<NUM_OF_SWITCHES;i++) 
      switch_msg_on[i][2]=i;
  }
  odDebugInit();
  usbInit();

  sendEmptyFrame = 0;

  sei();
	
  for (;;) {		/* main event loop */
    register char i;
    register char usb_busy;
    usb_busy = 0;
    wdt_reset();
    usbPoll();

    key = key_state;

    if (usbInterruptIsReady()) {
      //Process the switches
      for(i=0;(i<NUM_OF_SWITCHES) && (usb_busy==0);i++) {
	if((switch_state[i]==255) &&
	   (switch_sent[i]!=ON_SENT)) {
	  if (switch_msg_on[i][0]) {
	    switch_sent[i] = ON_SENT;
	    midiMsg[0] = switch_msg_on[i][0]; 
	    midiMsg[1] = switch_msg_on[i][1]; 
	    midiMsg[2] = switch_msg_on[i][2]; 
	    midiMsg[3] = switch_msg_on[i][3];
	    sendEmptyFrame = 0;
	    usbSetInterrupt(midiMsg, 4);
	    usb_busy = 1; 
	  }
	} else if((switch_state[i] == 0) &&
		  (switch_sent[i]!=OFF_SENT)) {
	  switch_sent[i] = OFF_SENT;
	  if (switch_msg_off[i][0]) {
	    midiMsg[0] = switch_msg_off[i][0];
	    midiMsg[1] = switch_msg_off[i][1];
	    midiMsg[2] = switch_msg_off[i][2];
	    midiMsg[3] = switch_msg_off[i][3];
	    sendEmptyFrame = 0;
	    usbSetInterrupt(midiMsg, 4);
	    usb_busy = 1;
	  }
	}
      }
      // State machine used to control the calibration mode
      switch(ctrl_state) {
      case ct_START:
	flasher = 0b10101010; //Fast flashing
	if (key_state>128) 
	  ctrl_state = ct_PRESSED_1;
	break;
      case ct_PRESSED_1:
	flasher = 0b11111111; //Remains ON
	if (key_state<128) {
	  val_cnt = 0;
	  max_val = 0;
	  ctrl_state = ct_GET_MAX;
	}
	break;
      case ct_GET_MAX:
	flasher = 0x0; //Remains OFF
	if (val_cnt < VAL_NUMBER) {
	  val=sensor();
	  if(val>-1) {
	    val_cnt++;
	    max_val+=val;
	  }    
	} else {
	  ctrl_state = ct_WAIT_PRESS_1;
	}
	break;
      case ct_WAIT_PRESS_1:
	flasher = 0b11001100; //Slower flashing
	if (key_state>128)
	  ctrl_state = ct_PRESSED_2;
	break;
      case ct_PRESSED_2:
        flasher = 0b11111111; //Remains ON
	if (key_state<128) {
	  val_cnt=0;
	  min_val=0;
	  ctrl_state = ct_GET_MIN;
	}
	break;
      case ct_GET_MIN:  
	flasher = 0x0; //Remains OFF
	if (val_cnt < VAL_NUMBER) {
	  val=sensor();
	  if(val>-1) {
	    val_cnt++;
	    min_val+=val;
	  }    
	} else {
          //Calculate the callibration coefficients
	  ctrl_state = ct_WORK;
          min_val = min_val / VAL_NUMBER;
          max_val = max_val / VAL_NUMBER;
	  if(min_val>max_val) {
	    tmp_val = min_val;
	    min_val = max_val;
            max_val = tmp_val;
          }
          val_tresh = min_val;
          min_val = (140l * 256l *256l) / (max_val - min_val);
	}
	break;
      case ct_WORK:
	if (key_state>128) {
	  ctrl_state = ct_PRESSED_1;
        }
        flasher = 0b10000000;
	//Process the capacitive sensor
	val = sensor();	// 0..1023
	if (val>-1) {
	  //New value
	  max_val = val-val_tresh;
	  max_val *= min_val;
	  val = max_val >> 16;
	  //Now cut the margins
	  if(val>6)
	    val -= 6;
	  else
	    val=0;
	  if(val>127)
	    val=127;
	  // In the line below we could add hystheresis, but 
	  // now hystheresis range is set to zero
	  // We send the new value only if USB is not busy
	  // (sending of switches state has higher priority)
	  if ((usb_busy==0) && 
	      ((sensorOld - val > 0) || (sensorOld - val < -0))){
	    sensorOld = val;
	    // MIDI CC msg
	    midiMsg[0] = pedal_msg[0]; //was 0x0b
	    midiMsg[1] = pedal_msg[1]; //was 0xb0
	    midiMsg[2] = pedal_msg[2];	// cc 70..77 
	    midiMsg[3] = pedal_msg[3] ? (127 - val) :  val;
	    sendEmptyFrame = 0;
	    usbSetInterrupt(midiMsg, 4);
	    break; //don't process other controls when sensor message is sent
	  }
	} 
	break;
      } //switch
    }		// usbInterruptIsReady()
  }
  return 0;
}
