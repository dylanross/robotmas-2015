/*
 * Send serial messages over an RF channel.
 */

#include <VirtualWire.h>


int boardID;
int RF_BAUD = 4000;

void setup() {
	// set board ID for communications
	boardID = 0;

	// set up wired serial line
	Serial.begin(9600);

	// set up RF serial TX
	vw_set_rx_pin(11);		// set receive pin for RF link
	vw_set_tx_pin(12);		// set transmit pin for RF link
	vw_setup(RF_BAUD);		// data transfer rate (bits per sec)
	vw_rx_start();			// start the receiver PLL
	
	// set up pins
	pinMode(13,OUTPUT);
}


void send(char *msg){
	// TODO append BOARD_ID to all outgoing messages
	Serial.print("T:");
	Serial.print(msg);
	digitalWrite(13, 1);
	vw_send((uint8_t *)msg, strlen(msg));
	vw_wait_tx(); // Wait until the whole message is gone
	digitalWrite(13,0);
}


void receive() {
	uint8_t buf[VW_MAX_MESSAGE_LEN];
	uint8_t buflen = VW_MAX_MESSAGE_LEN;
	
	Serial.print("R:");
	if (vw_get_message(buf, &buflen)) { // non-blocking
		// print whole lines to wired Serial line
		int i = 0;
		while ((char) buf[i] != *"\n") {
			Serial.print((char)buf[i]);
			i++;
		}
	}
	else {
		Serial.print("NO RESPONSE");
	}
	Serial.println(" ");
}


int DELAY_LONG = 1000;
int DELAY_SHORT = 100;
void loop(){
	// send messages over RF serial link
	// TODO remove explicit delays and implement listen-before-talk
//	send("0,P,2\n\r"); delay(DELAY_SHORT); receive();

	send("0,S,#1P600T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#2P600T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#3P600T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();

	send("0,S,#1P1500T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#2P1500T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#3P1500T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();

	send("0,S,#1P2400T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#2P2400T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,S,#3P2400T500\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
	send("0,P\n\r"); delay(DELAY_SHORT); receive();
}
