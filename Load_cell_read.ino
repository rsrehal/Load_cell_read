/* 
  Reads load cell values from strawberry cart
  
  Author: Richie Rehal
  Date: 8-15-2014
  Bio-Automation Lab
  
  Addition: April 2015
  Added in code ot read from custom IMU
*/

#define INTERVAL 100    //100ms intervals = 10 samples per second

int buttonPin = 20;        // corresponds to ISR.3 on Arduino Mega
int analogPin[4] = {0, 1, 2, 3};
int loadCellVal[4] = {0};

unsigned long currentTime = 0;
unsigned long prevTime = 0;
volatile int captureFlag = 0;

float accel[3] = {0};
float gyro[3] = {0};
float compass[3] = {0};
float euler[3] = {0};
float quatern[4] = {0};
float IMUvalues[16] = {0};

String IMUinput = "";

void timedOutput()
{      
      while(millis() - prevTime < INTERVAL)
      {
        //wait
      }  
}//timedOutput()

void capture()
{
   // ISR
   // when button is pressed, capture and transmit data
   captureFlag = 1;
   
}//capture()

void readLoadCells()
{
    float analogVoltage = 0;
    float totalVoltage = 0;
    unsigned short adcVal = 0;
    uint8_t upper = 0;
    uint8_t lower = 0;
/*
    Serial.println();
    Serial.println();
    Serial.println("Printing load cell readings...");
    Serial.println();
*/
    for(int i = 0; i < 4; i++)
    {
      loadCellVal[i] = analogRead(analogPin[i]);        


      // output data
      Serial.print("Load cell ");
      Serial.print(i+1);
      Serial.print(" , value = ");
      Serial.print(loadCellVal[i]);
 
      analogVoltage = ((float)loadCellVal[i] / 1024) * 5;
      Serial.print(" , \t "); 

      totalVoltage += analogVoltage; 
      Serial.print(analogVoltage, 3);
	//Serial.println(" V");
      Serial.println();
//


     adcVal = (unsigned short)loadCellVal[i];
     lower = adcVal & 0xFF;
     upper = adcVal >> 8;
     //Serial.write(adcVal);
/*      
     Serial.write(upper);
     Serial.write(lower);
*/    
     //Serial.write(0x0A);
     //Serial.write(0x0B);

    //Serial.print(loadCellVal[i], HEX);
    

    }//for i
    
    // Note: commma delimiters for easier Excel importing
    Serial.print("Total Voltage (V) = ,,, \t\t ");
    Serial.println(totalVoltage);
    totalVoltage = 0;    // reset voltage sum
    Serial.print("Time (ms) = ,,,, \t\t\t ");
    currentTime = millis();
    Serial.println(currentTime);
    //Serial.println();

}//readLoadCells()


// reads from IMU device through Serial2
void readMotionData(void)
{
  int i = 0;
  char inChar = 0;
  
  //first send byte indicating ready to recieve
  Serial2.print('r');
  currentTime = millis();
  
  //wait
  //delay(50);
  
  while(i < 16)
  {
    if(Serial2.available())
    {  
      inChar = Serial2.read();
      //delay(10);
      //Serial.print(inChar);
      
      if(inChar != '\n')
      {
        IMUinput += inChar;
      }else{
        //valid float found
        IMUvalues[i++] = IMUinput.toFloat();
        //Serial.println(IMUvalues[i-1]);
        IMUinput = "";
      }
    }
  }//while all data not received
     
  Serial2.flush();
  i = 0;
   
   // output data to Serial
   Serial.print("Accel X =, ");
   Serial.println(IMUvalues[0], 6);
   Serial.print("Accel Y =, ");
   Serial.println(IMUvalues[1], 6);
   Serial.print("Accel Z =, ");
   Serial.println(IMUvalues[2], 6);
   Serial.print("Roll =, ");
   Serial.println(IMUvalues[9], 6);
   Serial.print("Pitch =, ");
   Serial.println(IMUvalues[10], 6);
   Serial.print("Yaw =, ");
   Serial.println(IMUvalues[11], 6);
   Serial.print("Quaternion 1 =, ");
   Serial.println(IMUvalues[12], 6);
   Serial.print("Quaternion 2 =, ");
   Serial.println(IMUvalues[13], 6);
   Serial.print("Quaternion 3 =, ");
   Serial.println(IMUvalues[14], 6);
   Serial.print("Quaternion 4 =, ");
   Serial.println(IMUvalues[15], 6);
   Serial.print("Time (ms) =,, ");
   Serial.println(currentTime);

   Serial.println();
      
  
}//readMotionData()

void setup()
{
  
  //start serial
  Serial.begin(115200);
  
  Serial2.begin(115200);
  //Serial.println("Set up complete");
  
  //setup pins
  pinMode(buttonPin, INPUT);
  attachInterrupt(3, capture, LOW);  //interrupt on button press
                                    //button = active low
  
}//setup()

void loop()
{
  char RXbyte = 0;


  if(captureFlag == 1)
  {
      readLoadCells();
      
      //readMotionData();
      
      captureFlag = 0;
  }


/*  
  //currentTime = millis();
  
  //if(Serial.available() > 0)
  if(true)
  {
    RXbyte = Serial.read();
    
    if(RXbyte == 'r')
    //if(true)
    {      
      // read and write analog values

      readLoadCells();
      
    }//if 'r'
    else
    {
      //timed output

    }//else
  }//if
  
  //prevTime = millis();

*/  
  
}//loop()
