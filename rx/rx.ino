/*
 * Receive serial messages over an RF channel, parse them, and deliver to a
 * connected computer over wired serial.
 */

#include <VirtualWire.h>


int RF_BAUD = 4000;		 // baud rate for RF communications (bits per sec)
int BOARD_ID = 1;		 // identifier used for RF communications

const int N_SERVOS = 3;		 // number of servos connected to USC
int SERVO_TO_SC[] = {1, 2, 3};	 // maps servo indices (0, 1, 2) to USC address (1, 2, 3, ...)
int POS_TO_PIN[] = {A3, A4, A5}; // maps servo indices (0, 1, 2) to analog input pins (0, 1, 2, ...)
int POS_MAX[N_SERVOS];		 // maps servo indices (0, 1, 2) to maximum position reading
int POS_MIN[N_SERVOS];		 // maps servo indices (0, 1, 2) to minimum position reading

void setup() {
	// set up wired serial line
  	Serial.begin(9600);

	// set up RF serial RX
	vw_set_rx_pin(12);	// set receive pin for RF serial link
	vw_set_tx_pin(11);	// set transmit pin for RF serial link
	vw_setup(RF_BAUD);  	// set RF data transfer rate (bits per sec)
	vw_rx_start();       	// start receiver PLL

	// set up pins
	pinMode(13, OUTPUT);	// used as an indicator for debugging purposes
}


int sense_position(int servo_ID) {
	int pos = analogRead(POS_TO_PIN[servo_ID]);
	return pos;
}


void send(char *msg){
	// TODO append BOARD_ID to all outgoing messages
	digitalWrite(13, 1);			// turn indicator LED on
	vw_send((uint8_t *)msg, strlen(msg));	// send the message
	vw_wait_tx(); 				// wait for message to be sent
	digitalWrite(13, 0);			// turn indicator LED off
}


// received RF message format : "SENDER_ID,CMD_ID,CMD_BODY"
// indices :     	             0    1   2  3    4
int SENDER_ID_index = 0;	// index at which sender ID starts in received RF message
int CMD_ID_index = 2;		// index at which command ID starts in received RF message
int CMD_BODY_index = 4;		// index at which command body starts in received RF message

void parse(uint8_t buf[]) {
	int SENDER_ID = (int) buf[SENDER_ID_index];
	char CMD_ID = (char) buf[CMD_ID_index];

	if (SENDER_ID != BOARD_ID) {
		if (CMD_ID == *"S") {
			// servo command, format : "S,#1P1500T500"
			int i = CMD_BODY_index;
			while ((char) buf[i] != *"\n") {
				Serial.print((char)buf[i]);
				i++;
			}
			Serial.println(" ");
			send("1,s\n\r");
		}
		else if (CMD_ID == *"P") {
			// request position data, format : "P" or "P,{0 -- H}"
			if (buf[CMD_ID_index + 1] == *"\n") {
				// all position data has been requested
				// measure position of each servo
				int pos_arr[N_SERVOS];
				for (int i = 0; i < N_SERVOS; i++) {
					pos_arr[i] = sense_position(i);
				}

				// convert to char array for transmission
				char *msg = "";
				sprintf(msg, "1,p,%i,%i,%i\n\r", pos_arr[0], pos_arr[1], pos_arr[2]);

				// transmit measurements
				send(msg);
			}
			else {
				// specific position data has been requested
				// measure position of servo
				int servo_ID = (int) buf[CMD_BODY_index];
				int pos = sense_position(servo_ID);

				// convert to char array for transmission
				char *msg = "";
				sprintf(msg, "1,p,%i\n\r", pos);

				// transmit measurement
				send(msg);
			}
		}
		else if (CMD_ID == *"U") {
			// request ultrasonic range data, format : "U" or "U,{0 -- 3}"
			send("1,u,NI\n\r");
		}
		else if (CMD_ID == *"A") {
			// request accelerometer data, format : "A"
			send("1,a,NI\n\r");
		}
		else if (CMD_ID == *"G") {
			// request GPS data, format : "G"
			send("1,g,NI\n\r");
		}
		else {
			// received command does not match known responses!
			send("1,NR\n\r");
		}
	}
}


void loop() {
	// receive, parse, and execute commands sent over RF serial link
	uint8_t buf[VW_MAX_MESSAGE_LEN];
	uint8_t buflen = VW_MAX_MESSAGE_LEN;

	if (vw_get_message(buf, &buflen)) {
		parse(buf);
	}
}
