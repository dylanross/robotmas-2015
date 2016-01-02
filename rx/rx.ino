/*
 * Receive serial messages over RF channel, parse and execute any relevant
 * commands, then deliver appropriate response over the same RF channel. 
 */

#include <VirtualWire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// declare global variables and constants

// indicator pin
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// wired serial communications
int BAUD = 9600;			   // baud rate for wired serial communication (bits per sec)

// RF communications
int RF_BAUD = 4000;		 	   // baud rate for RF communications (bits per sec)
int BOARD_ID = 1;		 	   // identifier used for RF communications
const int RF_RX_PIN = 12;	 	   // pins for receive (RF) and transmit (TX) signal lines
const int RF_TX_PIN = 11;

// ultrasonic ranging
int nSensors = 4;                          // placeholder for global "number of sensors" variable
int ranges[4] = {0, 0, 0, 0};              // placeholder for input array

const int trigPin = 28;                    // initialise pin assignments for ultrasonic ranging
const int echoPin0 = 31;
const int echoPin1 = 30;
const int echoPin2 = 32;
const int echoPin3 = 33;

int echoPins[4] = {echoPin0, echoPin1, echoPin2, echoPin3}; // build array to store echo pins

// servomotors
const int N_SERVOS = 9;		 	   // number of servos connected to USC
int SERVO_TO_SC[] = {1, 2, 3, 4, 5, 6, 	   // maps servo indices (0, 1, 2, ..., 17) to USC addresses
		     7, 8, 9}; 		   // (1, 2, ..., 18)

// servo position sensing
int MUX_ADDR_A_PIN = 22;		   // multiplexer address pin A
int MUX_ADDR_B_PIN = 24;		   // multiplexer address pin B
int MUX_ADDR_C_PIN = 26;		   // multiplexer address pin C

int POS_TO_PIN[] = {A0, A0, A0, A0, A0, A0, A0, A0, 	// maps servo indices (0, 1, 2, ..., 17) to 
	            A1, A1, A1, A1, A1, A1, A1, A1, 	// analog input pins (A0, A1, A2) used for measuring
		    A2, A2, A2, A2, A2, A2, A2, A2};	// servo position

int POS_TO_ADDR_A[]  = {0, 1, 0, 1, 0, 1, 0, 1, 	// maps servo indices (0, 1, 2, ..., 17) to 
	                0, 1, 0, 1, 0, 1, 0, 1, 	// mux address A required to route position
			0, 1, 0, 1, 0, 1, 0, 1};	// signal to analog pin
int POS_TO_ADDR_B[]  = {0, 0, 1, 1, 0, 0, 1, 1, 	// maps servo indices (0, 1, 2, ..., 17) to
			0, 0, 1, 1, 0, 0, 1, 1, 	// mux address B required to route position
			0, 0, 1, 1, 0, 0, 1, 1};	// signal to analog pin
int POS_TO_ADDR_C[]  = {0, 0, 0, 0, 1, 1, 1, 1, 	// maps servo indices (0, 1, 2, ..., 17) to
			0, 0, 0, 0, 1, 1, 1, 1, 	// mux address C required to route position
			0, 0, 0, 0, 1, 1, 1, 1};	// signal to analog pin

// 6-axis accelerometer sensing
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


void setup() {
	// set up wired serial line
  	Serial.begin(BAUD);		// open serial line and set baud rate
    	while (!Serial); 		// wait for enumeration

	// set up indicator pin
	pinMode(LED_PIN, OUTPUT);	// used as an indicator for debugging purposes

	// set up RF serial RX
	vw_set_rx_pin(RF_RX_PIN);	// set receive pin for RF serial link
	vw_set_tx_pin(RF_TX_PIN);	// set transmit pin for RF serial link
	vw_setup(RF_BAUD);  		// set RF data transfer rate (bits per sec)
	vw_rx_start();       		// start receiver PLL

	// set up sensor mux address pins as outputs
	pinMode(MUX_ADDR_A_PIN, OUTPUT);
	pinMode(MUX_ADDR_B_PIN, OUTPUT);
	pinMode(MUX_ADDR_C_PIN, OUTPUT);
	digitalWrite(MUX_ADDR_A_PIN, 0);
	digitalWrite(MUX_ADDR_B_PIN, 0);
	digitalWrite(MUX_ADDR_C_PIN, 0);

	// set up 6-axis accelerometer
	// join I2C bus (I2Cdev library doesn't do this automatically)
    	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        	Wire.begin();
        	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        	Fastwire::setup(400, true);
    	#endif

    	// initialize device
//    	Serial.println(F("Initializing I2C devices..."));
    	mpu.initialize();

    	// verify connection
//    	Serial.println(F("Testing device connections..."));
//    	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : 
//					      F("MPU6050 connection failed"));

    	// wait for ready
//    	Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    	while (Serial.available() && Serial.read()); // empty buffer
//    	while (!Serial.available());                 // wait for data
//    	while (Serial.available() && Serial.read()); // empty buffer again

    	// load and configure the DMP
//    	Serial.println(F("Initializing DMP..."));
    	devStatus = mpu.dmpInitialize();

    	// supply your own gyro offsets here, scaled for min sensitivity
    	mpu.setXGyroOffset(220);
    	mpu.setYGyroOffset(76);
    	mpu.setZGyroOffset(-85);
    	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    	// make sure it worked (returns 0 if so)
    	if (devStatus == 0) {
        	// turn on the DMP, now that it's ready
//        	Serial.println(F("Enabling DMP..."));
        	mpu.setDMPEnabled(true);

        	// enable Arduino interrupt detection
//        	Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        	attachInterrupt(0, dmpDataReady, RISING);
        	mpuIntStatus = mpu.getIntStatus();

        	// set our DMP Ready flag so the main loop() function knows it's okay to use it
//        	Serial.println(F("DMP ready! Waiting for first interrupt..."));
        	dmpReady = true;

        	// get expected DMP packet size for later comparison
        	packetSize = mpu.dmpGetFIFOPacketSize();
    	} else {
        	// ERROR!
        	// 1 = initial memory load failed
        	// 2 = DMP configuration updates failed
        	// (if it's going to break, usually the code will be 1)
//        	Serial.print(F("DMP Initialization failed (code "));
//        	Serial.print(devStatus);
//        	Serial.println(F(")"));
    	}


}


// interrupt detection routine for 6-axis accelerometer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


int sense_position(int servo_ID) {	// accepts zero-indexed servo ID (0, 1, 2, ..., 17, 18) and
					// returns raw analog sample from position feedback signal
	// set MUX address to route appropriate position signal to analog input
      	digitalWrite(MUX_ADDR_A_PIN, POS_TO_ADDR_A[servo_ID]);
      	delayMicroseconds(10);
      	digitalWrite(MUX_ADDR_B_PIN, POS_TO_ADDR_B[servo_ID]);
      	delayMicroseconds(10);
      	digitalWrite(MUX_ADDR_C_PIN, POS_TO_ADDR_C[servo_ID]);
      	delayMicroseconds(10);

	// read position signal
	int pos = analogRead(POS_TO_PIN[servo_ID]);
	return pos;
}


int* sense_range(int* ranges) {    	   // accepts array of length "nSensors", and returns modified array 
				  	   // containing sensor values
  int sensorx = 0;

  for (sensorx = 0; sensorx < nSensors; sensorx++) { //loops over all array entries

    if (ranges[sensorx] == 0) {            // if entry is zero, takes no action (returns zero)
    }

    else {                                 // if entry is non-zero, returns range value for that sensor
      
      int duration = 0;                    // initialise variables
      int distance = 0;
      
      pinMode(trigPin, OUTPUT);            // define input and output pins
      pinMode(echoPins[sensorx], INPUT);
      
      digitalWrite(trigPin, LOW);          // leaves 10 microsecond LOW to ensure clean signal
      delayMicroseconds(10);
      digitalWrite(trigPin, HIGH);         // sends 10 microsecond pulse
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      
      duration = pulseIn(echoPins[sensorx], HIGH); //listens for response from echo pin
      distance = 0.5 * duration / 29;      // coverts duration to a distance in cm - integer under "duration" 
      					   // defines units - 29 gives cm
      ranges[sensorx] = distance;          // enters the distance into the array
      delay(10);                           // DEBUG attempt to remove spurious returns - seems to work
    }
  }
  return ranges;                           // returns modified array
}



void send(char *msg){
	// TODO append BOARD_ID to all outgoing messages
	digitalWrite(LED_PIN, 1);	   // turn indicator LED on
	vw_send((uint8_t *)msg, strlen(msg));	// send the message
	vw_wait_tx(); 				// wait for message to be sent
	digitalWrite(LED_PIN, 0);		// turn indicator LED off
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
			if (buf[CMD_ID_index + 1] != *",") {
				// all position data has been requested
				// measure position of each servo
				int pos_arr[N_SERVOS];
				for (int i = 0; i < N_SERVOS; i++) {
					pos_arr[i] = sense_position(i);
				}

				// convert to char array for transmission
				// don't use hardcoded number of position
				// readings; use N_SERVOS global variable
				// instead
				char *msg = "";
				sprintf(msg, "1,p,%i,%i,%i\n\r", pos_arr[0], pos_arr[1], pos_arr[2]);

				// transmit measurements
				send(msg);
			}
			else {
				// specific position data has been requested
				// measure position of servo
				int servo_ID = (int) (buf[CMD_BODY_index] - '0');
				int pos = sense_position(servo_ID);

				// convert to char array for transmission
				char *msg = "";
				sprintf(msg, "1,p,%i,%i\n\r", servo_ID, pos);

				// transmit measurement
				send(msg);
			}
		}
		else if (CMD_ID == *"U") {
			// request ultrasonic range data, format : "U" or "U,{0 -- 3}"
			// TODO remove hard-coded references to number of
			// ultrasonic range sensors
			if (buf[CMD_ID_index + 1] == *"\n") {
				// all ultrasonic ranging data has been requested
				// take us measurements
				int ranges[4] = {1, 1, 1, 1};
				sense_range(ranges);

				// convert to char array for transmission
				char *msg = "";
				sprintf(msg, "1,u,%i,%i,%i,%i\n\r", ranges[0], ranges[1], 
								    ranges[2], ranges[3]);

				// transmit measurements
				send(msg);
			}
			else {
				// specific ultrasonic sensor data has been requested
				// take us measurement
				int us_ID = int(buf[CMD_BODY_index] - '0');
				int ranges[4] = {0, 0, 0, 0};
				ranges[us_ID] = 1;
				sense_range(ranges);

				// convert to char for transmission
				char *msg = "";
				sprintf(msg, "1,u,%i\n\r", ranges[us_ID]);

				// transmit measurements
				send(msg);
			}
		}
		else if (CMD_ID == *"A") {
			// request accelerometer data, format : "A"
			send("1,a,KHARN\n\r");
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
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetEuler(euler, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);


//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif
//
//        #ifdef OUTPUT_READABLE_YAWPITCHROLL
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
//        #endif
//
//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
    }

	uint8_t buf[VW_MAX_MESSAGE_LEN];
	uint8_t buflen = VW_MAX_MESSAGE_LEN;

	if (vw_get_message(buf, &buflen)) {
		parse(buf);
	}
}
