// Buffer to store incoming commands from serial port
String inData;

void setup() {
    Serial.begin(9600);
}

void loop() {
    while (Serial.available() > 0)
    {
        char recieved = Serial.read();
        inData += recieved; 

        // Process message when new line character is recieved
        if (recieved == '\n')
        {
            Serial.print("Arduino Received: ");
            Serial.print(inData);

            inData = ""; // Clear recieved buffer
        }
    }
}
