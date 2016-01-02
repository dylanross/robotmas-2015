/*
 * Send serial messages over an RF channel.
 */

#include <VirtualWire.h>


int BOARD_ID;
int RF_BAUD = 4000;

void setup() {
	// set board ID for communications
	BOARD_ID = 0;

	// set up wired serial line
	Serial.begin(115200);

	// set up RF serial TX
	vw_set_rx_pin(11);		// set receive pin for RF link
	vw_set_tx_pin(12);		// set transmit pin for RF link
	vw_setup(RF_BAUD);		// data transfer rate (bits per sec)
	vw_rx_start();			// start the receiver PLL
	
	// set up pins
	pinMode(13,OUTPUT);
}


void send(char *msg){
	// send a message over the RF serial line
	// TODO append BOARD_ID to all outgoing messages
//	Serial.print("T:");
//	Serial.print(msg);
	digitalWrite(13, 1);
	vw_send((uint8_t *)msg, strlen(msg));
	vw_wait_tx(); // Wait until the whole message is gone
	digitalWrite(13,0);
}


void receive() {
	// receive a message over the RF serial line
	uint8_t buf[VW_MAX_MESSAGE_LEN];
	uint8_t buflen = VW_MAX_MESSAGE_LEN;
	
//	Serial.print("R:");
	if (vw_get_message(buf, &buflen)) { // non-blocking
		// print whole lines to wired Serial line
		int i = 0;
		while ((char) buf[i] != *"\n") {
			Serial.print((char)buf[i]);
			i++;
		}
		Serial.println(" ");
	}
	else {
//		Serial.print("NO RESPONSE");
	}
}


//int DELAY_LONG = 1000;
//int DELAY_SHORT = 200;
int RESPONSE_DELAY = 1;
String inData;
void loop(){
	// receive messages over wired serial link, then forward over RF serial link,
	// collect response over RF serial link, and forward over wired serial link
	while (Serial.available() > 0)
	{
        	char recieved = Serial.read();  // store character received over wired serial line
        	inData += recieved; 		// append received character to string inData

        	if (recieved == '\r')		// a complete message has arrived
        	{
			char msg[inData.length()];
			inData.toCharArray(msg, inData.length());

            		send(msg);		// forward message over RF serial line

            		inData = ""; 		// clear input string
        	}
		receive();		// receive responses from sent messages
    	}
	receive();			// receive responses from sent messages
}
