// Serial command to change PID Settings during runtime
//    <Setpoint, Kp, Ki, Kd>

//    <37,150,1.3,0> //untested values


// Serial communication from https://forum.arduino.cc/index.php?topic=396450.0
// PID library from  https://playground.arduino.cc/Code/PIDLibrary/
// Adafruit library from https://github.com/adafruit/Adafruit_MAX31856

//============================================================
#include <Adafruit_MAX31856.h>
#include <PID_v1.h>
//============================================================
// Set up PID variables and object
double Setpoint = 25.0;
float Kp = 150;
float Ki = 1.3;
float Kd = 0.0;
double Input = 25.0;
double Output = 0;
PID ChipHeater(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float Kp2 = 150;
float Ki2 = 1.3;
float Kd2 = 0.0;
double Input2 = 25.0;
double Output2 = 0;
PID AdditionalHeater(&Input2, &Output2, &Setpoint, Kp2, Ki2, Kd2, DIRECT);

bool overshoot = false;
double OvershootTemperature = 26.0;
double OvershootThreashold  = 0.5;
//============================================================

// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 12, 13);
//============================================================

// use hardware SPI, just pass in the CS pin
Adafruit_MAX31856 additionalTempProbe = Adafruit_MAX31856(9);
Adafruit_MAX31856 heaterTemp = Adafruit_MAX31856(10);
//============================================================

//Set up communication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;
//============================================================


void setup() 
{
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Heat Stage Controll with 2 Thermocouples A10 is controlling PWM6 and A9 is controlling PWM11");

  //============================================================

  ChipHeater.SetMode(AUTOMATIC);
  ChipHeater.SetOutputLimits(0, 255);
  ChipHeater.SetSampleTime(2000);
  
  AdditionalHeater.SetMode(AUTOMATIC);
  AdditionalHeater.SetOutputLimits(0, 255);
  AdditionalHeater.SetSampleTime(2000);

  //============================================================

  if (!heaterTemp.begin()) 
  {
    Serial.println("Could not initialize thermocouple A10.");
    while (1) delay(10);
  }
  heaterTemp.setThermocoupleType(MAX31856_TCTYPE_K);
  heaterTemp.setConversionMode(MAX31856_ONESHOT_NOWAIT);

  if (!additionalTempProbe.begin()) 
  {
    Serial.println("Could not initialize thermocouple A9.");
    while (1) delay(10);
  }
  additionalTempProbe.setThermocoupleType(MAX31856_TCTYPE_K);
  additionalTempProbe.setConversionMode(MAX31856_ONESHOT_NOWAIT);
  
   
}

void loop() {
  //============================================================
  //
  // trigger a conversion
  heaterTemp.triggerOneShot();
  additionalTempProbe.triggerOneShot();
  
  //============================================================
  //read serial input
  recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
        ChipHeater.SetTunings(Kp, Ki, Kd); 
    }

  //============================================================

  //
  //
  delay(500); // time can be used for other tasks
  //
  //

  // check for conversion complete and read temperature
  if (heaterTemp.conversionComplete()) {
    Input = heaterTemp.readThermocoupleTemperature();
    ChipHeater.Compute();

    if (Input>OvershootTemperature)
    {
      overshoot = true;
    }
    else if (Input<Setpoint)
    {
      overshoot = false;
    }

    if (overshoot)
    {
      Output = 0;
    }


    
    analogWrite(6, Output); // PWM pin 6 on the short edge of the ItsyBitsy board
    Serial.print(Input);
  } else {
    Serial.print("Conversion not complete!");
  }

  Serial.print(", ");

    
  if (additionalTempProbe.conversionComplete()) {
	Input2 = additionalTempProbe.readThermocoupleTemperature();
    AdditionalHeater.Compute();
    analogWrite(11, Output2); // PWM pin 11 on the long edge of the ItsyBitsy board
	  
    Serial.print(Input2);
	
	
  } else {
    Serial.print("Conversion not complete!");
  }
  
   
  Serial.print(", ");  
  Serial.print(Output);
  Serial.print(",");
  Serial.print(Output2);
  Serial.print(",");
  Serial.println(Setpoint);
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    Setpoint = atof(strtokIndx); // copy it to messageFromPC
    OvershootTemperature = Setpoint + OvershootThreashold;
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Kp = atof(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    Ki = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    Kd = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    Serial.println(Setpoint);
    Serial.println(Kp);
    Serial.println(Ki);
    Serial.println(Kd);
}
