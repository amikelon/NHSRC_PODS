
//
//    FILE: SDI-12.001.pde
//  AUTHOR: Rob Tillaart
//    DATE: 2012-mar-18
//
// PUPROSE: talk to SDI-12 anemometer
//

#include <SoftwareSerial.h>

#define txPin 30
#define rxPin 11
#define INVERTED true          //SDI-12 uses inverted logic
#define HPRelay 27 //Outpin PIN to control HydraProbe 12V Rail
String change_addr = "Ab!";
String serial_ID = "0I!";
String Reading_0 = "0M!";
SoftwareSerial sdiAnem(rxPin, txPin, INVERTED);


void setup()
{
// communication with PC
Serial.begin(9600);
Serial.println("\nSDI-12 0.01");
pinMode(HPRelay, OUTPUT); //Set up HydraProbe relay
delay (50);
digitalWrite(HPRelay, HIGH); //turn on HydraProbe 12V Rail
delay(15);
// software serial for the sensor
sdiAnem.begin(1200);

delay(9);
digitalWrite(txPin, HIGH);
delay(13);
digitalWrite(txPin, LOW);
delay(9);


delay(9);
digitalWrite(txPin, HIGH);
delay(13);
digitalWrite(txPin, LOW);
delay(9);


char incomingAnswer[80];
char a;
int idy = 0;
delay(9);
digitalWrite(txPin, HIGH);
delay(13);
digitalWrite(txPin, LOW);
delay(9);
Serial.println("H");
sdiAnem.write("0SU!");
  if (sdiAnem.available() > 0)
  {
    a = sdiAnem.read();
    incomingAnswer[idy] = a & 0x7F;  // strip bit 8
    idy++;
  }
Serial.print("Reported settings are: ");  
Serial.println(incomingAnswer);
 

}

void loop()
{
  while(Serial.available() ==0);
  char commands = Serial.read();
char *answer;
if (commands == 'a'){
answer = addressQuery(); // char, no string
Serial.print("\n addressQuery: ");
Serial.println(answer);
}
else if (commands == 'b'){
AcknowledgeActive('0'); // char, no string
Serial.print("\n Address: ");
Serial.println(answer);
}
else if (commands == 'c'){
sendIdentification('0'); // char, no string
Serial.print("\n id: ");
Serial.println(answer);
}
//int t = 0;
//int n = 0;

/* startMeasurement('0', &t, &n);
Serial.print("\n startMeasurement: ");
Serial.println(answer);
Serial.println(t, DEC);
Serial.println(n, DEC);

delay(t*1000UL); // wait for the measurement to be ready
*/
else if (commands == 'd'){
sendData('0');
Serial.print("\n sendData: ");
Serial.println(answer);
}
else{
Serial.println("No Command");  
  }

delay(10000);
}

/////////////////////////////////////////////////////////
//
// Proto  SDI-12 commands
//
// names of functions, Table 5, page 8, SDI-12 1.3 specification
//


/////////////////////////////////////////////////////////
//
// Private members
//
char response[80];    // at least 35/75 according to 1.3 spec

/////////////////////////////////////////////////////////
//
// Public part I (admin commands)
//
char* AcknowledgeActive(char address)
{
char command[] = "a!";
command[0] = address;
request(2, command, response);
return response;
}

char* addressQuery()
{
  Serial.print("H-add");
char command[] = "?!";
request(2, command, response);
return response;
}

char* addressChange(char oldAddress, char newAddress)
{
char command[] = "aAb!";
command[0] = oldAddress;
command[2] = newAddress;
request(4, command, response);
return response;
}

char* sendIdentification(char address)
{
char command[] = "aI!";
command[0] = address;
request(3, command, response);
return response;
}

void printIdentification(char *idStr)
{
// TODO
}

/////////////////////////////////////////////////////////
//
// Public part II (measurements)
//
char* startMeasurement(char address, int *t, int *n)
{
char command[] = "aI!";
command[0] = address;
request(3, command, response);

int len = strlen(response);
*n = (int) (response[len-1] - '\0');
*t = atoi(&response[1]) / 10;

return response;
}

char* sendData(char address)
{
char command[] = "0R1!";
request(4, command, response);
return response;
}


/////////////////////////////////////////////////////////
//
// Private part I
//
void request(uint8_t length, char *command, char *response)
{
// send command
Serial.println("request");
/*delay(9);
digitalWrite(txPin, HIGH);
delay(13);
digitalWrite(txPin, LOW);
delay(9);
*/
Serial.println("index");
for (int idx=0; idx<length; idx++)
{
  sdiAnem.write(command[idx]);
}

// blocking until answer comes in
char c = ' ';
int idx = 0;
while (c != 10)  // answer ends with LineFeed = char 10
{
  Serial.println("reading SDI bus");
  if (sdiAnem.available() > 0)
  {
    c = sdiAnem.read();
    response[idx] = c & 0x7F;  // strip bit 8
    idx++;
  }
}
response[idx-2] = 0; // remove the CRLF and end string correctly
}

// --- END OF FILE ---
