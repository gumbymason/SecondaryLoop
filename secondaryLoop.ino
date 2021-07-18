/*
  Flow Sensor Variables
 */
byte statusLed    = 13;
byte sensorInterrupt = 0;         // 0 = digital pin 2
byte sensorPin       = 2;
float calibrationFactor = 5;      //12 pulses/litre according to seller.
volatile byte pulseCount;  
float flowRate;
unsigned long oldTime;

/*
  Pressure Sensor Variables
*/
float pressure_sensor[4];
  
//int val;                          // A simple data logger for the Arduino analog pins  
int sensorVal;

/*
  Thermocouple Includes, Defines, and Variables
*/
#include <string.h> //Use the string Library
#include <ctype.h>
#include <EEPROM.h>

#define PINEN 7 //Mux Enable pin
#define PINA0 4 //Mux Address 0 pin
#define PINA1 5 //Mux Address 1 pin
#define PINA2 6 //Mux Address 2 pin
#define PINSO 12 //TCAmp Slave Out pin (MISO)
#define PINSC 13 //TCAmp Serial Clock (SCK)
#define PINCS 9  //TCAmp Chip Select Change this to match the position of the Chip Select Link

int Temp[8], SensorFail[8];
float floatTemp[4];
char failMode[8];
int internalTemp, intTempFrac;
unsigned int Mask;
char i, j, UpdateDelay;
char Rxchar, Rxenable, Rxptr, Cmdcomplete, R;
char Rxbuf[32];
char adrbuf[3], cmdbuf[3], valbuf[12];
int Param;     
unsigned long time;


char num_thermo_sensors = 4;        // Change this value to the total number of thermocouple sensors in use. We are using 4.
int num_pressure_sensors = 4;       // Change this value to the total number of pressure sensors in use. We are using 4.

void setup()
{
  Serial.begin(9600);             // Initialize a serial connection for reporting values to the host
  pinMode(statusLed, OUTPUT);     // Set up the status LED line as an output
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  oldTime           = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

  //Serial.println("TCMUXV3");
  if (EEPROM.read(511)==1)
  {
    num_thermo_sensors = EEPROM.read(0);
    UpdateDelay = EEPROM.read(1);
  }
  pinMode(PINEN, OUTPUT);     
  pinMode(PINA0, OUTPUT);    
  pinMode(PINA1, OUTPUT);    
  pinMode(PINA2, OUTPUT);    
  pinMode(PINSO, INPUT);    
  pinMode(PINCS, OUTPUT);    
  pinMode(PINSC, OUTPUT);    
  
  digitalWrite(PINEN, HIGH);   // enable on
  digitalWrite(PINA0, LOW); // low, low, low = channel 1
  digitalWrite(PINA1, LOW); 
  digitalWrite(PINA2, LOW); 
  digitalWrite(PINSC, LOW); //put clock in low
}

/**
 * Main program loop
 */
void loop()
{ 
  if((millis() - oldTime) > 1000)             // Only process counters once per second
  { 
    // Start Flow Sensor Code
    detachInterrupt(sensorInterrupt);         //disable interrupts to calculate flow rate.
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;  // get time to calculate flow rate
    oldTime = millis();                       //interrupts are disabled so millis() won't increment correctly but it is close.
    unsigned int frac;
    pulseCount = 0;                           // Reset the pulse counter so we can start incrementing again
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING); // Enable the interrupt again now that we've finished sending output
    // End Flow Sensor Code

    // Start Pressure Code
    for(int n = 0; n < num_pressure_sensors; n++)
    {
      switch(n)
      {
        case 0: 
          sensorVal=analogRead(A1);
          break;
        case 1:
          sensorVal=analogRead(A2);
          break;
        case 2: 
          sensorVal=analogRead(A3);
          break;
        case 3: 
          sensorVal=analogRead(A4);
          break;
        default:
          sensorVal=analogRead(A1);
          break;
      }

      float voltage = (sensorVal*5.0)/1024.0;

      if(n == 0)
        pressure_sensor[0] = (((float)voltage-.5))*25;                       //calibrate here
      if(n == 1)
        pressure_sensor[1] = (((float)voltage-.5))*25;
      if(n == 2)
        pressure_sensor[2] = (((float)voltage-.5))*25;
      if(n == 3)
        pressure_sensor[3] = (((float)voltage-.5))*25;
    }
    // End Pressure code

    // Start Thermocouple code
    if(j<(num_thermo_sensors-1)) 
    {
      j++;
    }
    else 
    {
      j=0;
    }
      
    switch (j) //select channel
    {
      case 0:
        digitalWrite(PINA0, LOW); 
        digitalWrite(PINA1, LOW); 
        digitalWrite(PINA2, LOW);
        break;
      case 1:
        digitalWrite(PINA0, HIGH); 
        digitalWrite(PINA1, LOW); 
        digitalWrite(PINA2, LOW);
        break;
      case 2:
        digitalWrite(PINA0, LOW); 
        digitalWrite(PINA1, HIGH); 
        digitalWrite(PINA2, LOW);
        break;
      case 3:
        digitalWrite(PINA0, HIGH); 
        digitalWrite(PINA1, HIGH); 
        digitalWrite(PINA2, LOW);
        break;
      case 4:
        digitalWrite(PINA0, LOW); 
        digitalWrite(PINA1, LOW); 
        digitalWrite(PINA2, HIGH);
        break;
      case 5:
        digitalWrite(PINA0, HIGH); 
        digitalWrite(PINA1, LOW); 
        digitalWrite(PINA2, HIGH);
        break;
      case 6:
        digitalWrite(PINA0, LOW); 
        digitalWrite(PINA1, HIGH); 
        digitalWrite(PINA2, HIGH);
        break;
      case 7:
        digitalWrite(PINA0, HIGH); 
        digitalWrite(PINA1, HIGH); 
        digitalWrite(PINA2, HIGH);
        break;
    }
      
    delay(5);
    digitalWrite(PINCS, LOW); //stop conversion
    delay(5);
    digitalWrite(PINCS, HIGH); //begin conversion
    delay(100);  //wait 100 ms for conversion to complete
    digitalWrite(PINCS, LOW); //stop conversion, start serial interface
    delay(1);
      
    Temp[j] = 0;
    failMode[j] = 0;
    SensorFail[j] = 0;
    internalTemp = 0;
    for (i=31;i>=0;i--)
    {
        digitalWrite(PINSC, HIGH);
        delay(1);
        
         //print out bits
       #ifdef SHOWMEYOURBITS
       if (digitalRead(PINSO)==1)
        {
          Serial.print("1");
        }
        else
        {
          Serial.print("0");
        }
        #endif
        
      if ((i<=31) && (i>=18))
      {
        // these 14 bits are the thermocouple temperature data
        // bit 31 sign
        // bit 30 MSB = 2^10
        // bit 18 LSB = 2^-2 (0.25 degC)
        
        Mask = 1<<(i-18);
        if (digitalRead(PINSO)==1)
        {
          if (i == 31)
          {
            Temp[j] += (0b11<<14);//pad the temp with the bit 31 value so we can read negative values correctly
          }
          Temp[j] += Mask;
        }
      }
      //bit 17 is reserved
      //bit 16 is sensor fault
      if (i==16)
      {
        SensorFail[j] = digitalRead(PINSO);
      }
      
      if ((i<=15) && (i>=4))
      {
        //these 12 bits are the internal temp of the chip
        //bit 15 sign
        //bit 14 MSB = 2^6
        //bit 4 LSB = 2^-4 (0.0625 degC)
        Mask = 1<<(i-4);
        if (digitalRead(PINSO)==1)
        {
          if (i == 15)
          {
            internalTemp += (0b1111<<12);//pad the temp with the bit 31 value so we can read negative values correctly
          }
          
          internalTemp += Mask;//should probably pad the temp with the bit 15 value so we can read negative values correctly
        }          
      }
      //bit 3 is reserved
      if (i==2)
      {
        failMode[j] += digitalRead(PINSO)<<2;//bit 2 is set if shorted to VCC
      }
      if (i==1)
      {
        failMode[j] += digitalRead(PINSO)<<1;//bit 1 is set if shorted to GND
      }
      if (i==0)
      {
        failMode[j] += digitalRead(PINSO)<<0;//bit 0 is set if open circuit
      }
      digitalWrite(PINSC, LOW);
      delay(1);
    }
    
    if (SensorFail[j] == 1)
    {
      Serial.print("FAIL");
      if ((failMode[j] & 0b0100) == 0b0100)
      {
        Serial.println(" SHORT TO VCC");
      }
      if ((failMode[j] & 0b0010) == 0b0010)
      {
        Serial.println(" SHORT TO GND");
      }
      if ((failMode[j] & 0b0001) == 0b0001)
      {
        Serial.println(" OPEN CIRCUIT");
      }
    }
    else
    {
      floatTemp[j] = (float)Temp[j] * 0.25;
    }//end reading sensors
    
    if (Serial.available() > 0)    // Is a character waiting in the buffer?
    {
      Rxchar = Serial.read();      // Get the waiting character

      if (Rxchar == '@')      // Can start recording after @ symbol
      {
        if (Cmdcomplete != 1)
        {
          Rxenable = 1;
          Rxptr = 1;
        }//end cmdcomplete
      }//end rxchar
      if (Rxenable == 1)           // its enabled so record the characters
      {
        if ((Rxchar != 32) && (Rxchar != '@')) //dont save the spaces or @ symbol
        {
          Rxbuf[Rxptr] = Rxchar;
          Rxptr++;
          if (Rxptr > 13) 
          {
            Rxenable = 0;
          }//end rxptr
        }//end rxchar
        if (Rxchar == 13) 
        {
          Rxenable = 0;
          Cmdcomplete = 1;
        }//end rxchar
      }//end rxenable
    }// end serial available


   
    if (Cmdcomplete == 1)
    {
      Cmdcomplete = 0;
      cmdbuf[0] = toupper(Rxbuf[1]); //copy and convert to upper case
      cmdbuf[1] = toupper(Rxbuf[2]); //copy and convert to upper case
      cmdbuf[2] = 0; //null terminate        Command = Chr(rxbuf(3)) + Chr(rxbuf(4))
      valbuf[0] = Rxbuf[3]; //        Mystr = Chr(rxbuf(5))
      R = Rxptr - 1;
      for (i = 4 ; i <= R ; i++)//For I = 6 To R
      {
        valbuf[i-3] = Rxbuf[i]; //Mystr = Mystr + Chr(rxbuf(i))
      }
      valbuf[R+1] = 0; //null terminate
      Param = atoi(valbuf);//   Param = Val(mystr)
      if (strcmp(cmdbuf,"NS")==0)       //num_thermo_sensors
      {
        //'Print "command was ON"
        if ((Param <= 8) && (Param > 0)) 
        {
          num_thermo_sensors = Param;                   
        }     
      }
      if (strcmp(cmdbuf,"UD")==0)       //UpdateDelay
      {
        //'Print "command was ON"
        if ((Param <= 60) && (Param >= 0)) 
        {
          UpdateDelay = Param;                   
        }
      }
      if (strcmp(cmdbuf,"SV")==0)       //Save
      {
        EEPROM.write(0,num_thermo_sensors);
        EEPROM.write(1,UpdateDelay);
        EEPROM.write(511,1);
      }
    }
    toExcel();
  }
}

/*
 * Interrupt service Routine
 */
void pulseCounter()
{
  pulseCount++; // Increment the pulse counter
}

/*
 * Display measurements to the excel sheet
 */
void toExcel()
{
  // Print the flow rate for this second in litres / minute
  Serial.print(int(flowRate));      // Print the integer part of the variable
  Serial.print(", ");                // need a comma delimiter
  // Print the Pressure in PSI
  for(int i = 0; i < num_pressure_sensors; i++)
  {
    Serial.print (pressure_sensor[i]);
    Serial.print(", ");
  }
  
  for(int i = 0; i < num_thermo_sensors; i++)
  {
    Serial.print(floatTemp[i],2);
    Serial.print(", ");
  }

  Serial.println();
}