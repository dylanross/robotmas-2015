void setup() {
  Serial.begin(9600);
}

int nSensors = 4;                          //placeholder for global "number of sensors" variable
int Ranges[4] = {1, 1, 1, 1};              //placeholder for input array

const int trigPin = 28;                    //initialise pin assignments
const int echoPin0 = 31;
const int echoPin1 = 30;
const int echoPin2 = 32;
const int echoPin3 = 33;

int echoPins[4] = {echoPin0, echoPin1, echoPin2, echoPin3}; //build array to store echo pins



int* myRangesFx(int* ranges) {    //accepts array of length "nSensors", and returns modified array 
				  // containing sensor values

  int sensorx = 0;

  for (sensorx = 0; sensorx < nSensors; sensorx++) { //loops over all array entries

    if (ranges[sensorx] == 0) {            //if entry is zero, takes no action (returns zero)
    }

    else {                                 //if entry is non-zero, returns range value for that sensor
      
      int duration = 0;                    //initialise variables
      int distance = 0;
      
      pinMode(trigPin, OUTPUT);            //define input and output pins
      pinMode(echoPins[sensorx], INPUT);
      
      digitalWrite(trigPin, LOW);          //leaves 10 microsecond LOW to ensure clean signal
      delayMicroseconds(10);
      digitalWrite(trigPin, HIGH);         //sends 10 microsecond pulse
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      
      duration = pulseIn(echoPins[sensorx], HIGH); //listens for response from echo pin
      distance = 0.5 * duration / 29;      //coverts duration to a distance in cm - integer under "duration" 
      					   //defines units - 29 gives cm
      ranges[sensorx] = distance;          //enters the distance into the array
      delay(10);                           //DEBUG attempt to remove spurious returns - seems to work
    }
  }
  return ranges;                           //returns modified array

}


//test code - prints the input and output arrays
//modify the values in "Ranges" for testing purposes
void loop() {
  int Ranges[4] = {1, 1, 1, 1};
  Serial.print("\ninput:  ");
  Serial.print(Ranges[0]);
  Serial.print("/");
  Serial.print(Ranges[1]);
  Serial.print("/");
  Serial.print(Ranges[2]);
  Serial.print("/");
  Serial.print(Ranges[3]);
  Serial.print("\noutput: ");
  int* result = myRangesFx(Ranges);
  Serial.print(result[0]);
  Serial.print("/");
  Serial.print(result[1]);
  Serial.print("/");
  Serial.print(result[2]);
  Serial.print("/");
  Serial.print(result[3]);
  delay(1000);
}
