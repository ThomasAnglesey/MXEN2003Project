//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Robot.h"
#define DEBOUNCE 20
#define FRONT_THRESHOLD 100
#define SIDE_THRESHOLD 50
  
volatile bool robotState = 0;
volatile bool gripperState = 0;



//static function prototypes, functions only called in this file

int main(void)
{ 
  cli();

  DDRL |= (1<<DDL0)|(1<<DDL1)|(1<<DDL2);        // put L0-L1 into low impedance output mode
  DDRA |= (1<DDA0)|(1<<DDA1);                   // put A0-A1 into low impedance output mode
  PORTL |= 0xFF;                                // set PORTL to output
  PORTA |= 0xFF;                                // set PORTA to output
  DDRD = 0;                                     //PORTD as input
  //PORTD |= (1<<PD2);                          // pullup
  //EIMSK |= (1<<INT2);                         // enable int0
  //EICRA |= (1<<ISC21);                        // falling edge

  // motor PWM
  TCCR1A = (1<<COM1A1) | (1<<COM1B1);         // enable PWM on OC1A    
  TCCR1B |= (1<<WGM13)|(1<<CS11);             // set WGM13 mode 8 (PWM mode) and Prescaler 8
  ICR1 = 10000;                               // TOP value for 100Hz frequency on Timer
  TCNT1 = 0;                                  // start Timer1 at 0
  DDRB |= (1<<PB5)|(1<<PB6);                  // OCR1A output PWM
  OCR1A = 0;                                  // Speed of right motor
  OCR1B = 0;                                  // speed of left motor

  // servo motor PWM
  TCCR3A = (1<<COM3A1);                     // enable PWM on OC3A    
  TCCR3B |= (1<<WGM33)|(1<<CS31);           // set WGM13 mode 8 (PWM mode) and Prescaler 8
  ICR3 = 20000;
  TCNT3 = 0;                                // start Timer3 at 0
  DDRE |= (1<<PE3);
  OCR3A = 1200;                             // speed of servo

  serial0_init();
  serial2_init();
  adc_init();
  milliseconds_init();

  uint8_t recievedData[6]; //recieved data array
  uint16_t combinedDatax;
  uint16_t combinedDatay;
  //uint16_t combinedDataz;
  uint8_t databyte1 = 0;
  uint8_t databyte2 = 0;
  uint8_t databyte3 = 0;
  uint8_t databyte4 = 0;
  uint8_t databyte5 = 0;
  uint8_t databyte6 = 0;
  uint32_t current_ms;
  uint32_t last_send_ms;
  char serial_string[60] = {0}; // String used for printing to terminal
  static int16_t leftMotor = 0;
  static int16_t rightMotor = 0;
  static int16_t fc;
  static int16_t rc;
  static uint16_t leftMotorSpeed;
  static uint16_t rightMotorSpeed;
  uint16_t batteryVoltage;
  uint16_t frontSensor;
  uint16_t leftSensor;
  uint16_t rightSensor;
  uint16_t leftLDR;
  uint16_t rightLDR;
  uint16_t frequency;
  uint32_t freqTimeNow;
  uint32_t freqTimeBefore = 0;
  uint16_t freqTrackerLeft = 0;
  uint16_t freqTrackerRight = 0;

  uint16_t getLongDistance(uint16_t adcInput);
  uint16_t getShortDistance(uint16_t adcInput);
  void turnRight(uint16_t turnTime);
  void turnLeft(uint16_t turnTime);
  void goForward();
  void adjustLeft();
  void adjustRight();

  sei();

  while(1)//main loop
  {
    current_ms = milliseconds_now();
    freqTimeNow = milliseconds_now();
    //batteryVoltage = adc_read(0);
    batteryVoltage = (2*(uint32_t)adc_read(3)*5000)/1023;
    // sprintf(serial_string,"\nVoltage: %3umV", batteryVoltage);
    // serial0_print_string(serial_string);
    
    frontSensor =  getLongDistance(adc_read(15));
    // sprintf(serial_string, "\nFront Distance: %4umm", frontSensor);
    // serial0_print_string(serial_string);
    // if(frontSensor < FRONT_THRESHOLD)
    // {
    // serial0_print_string("too close front");
    // }
    leftSensor = getShortDistance(adc_read(14));
    // sprintf(serial_string, "\nLeft Distance: %4umm", leftSensor);
    // serial0_print_string(serial_string);
    // if(leftSensor < SIDE_THRESHOLD)
    // {
    //    serial0_print_string("too close left");
    // }
    rightSensor = getShortDistance(adc_read(13));
    // sprintf(serial_string, "\nRight Distance: %4umm", rightSensor);
    // serial0_print_string(serial_string);
    // if(rightSensor < SIDE_THRESHOLD)
    // {
    // serial0_print_string("too close right");
    // }
    
    
    leftLDR = adc_read(1);
    rightLDR = adc_read(2);

    // sprintf(serial_string,"\nLeft LDR: %3u, Right LDR: %3u", leftLDR, rightLDR); //Format string
    // serial0_print_string(serial_string);



    if(leftLDR < 100)
    {
      uint32_t currentTime = milliseconds_now();
      static uint32_t previousTime = 0;
      if((currentTime - previousTime) > DEBOUNCE)
        {
          freqTrackerLeft += 1;
          //serial0_print_string("\nadd 1");
        }
        previousTime = currentTime;
    }
    else if(rightLDR < 100)
    {
      uint32_t currentTime = milliseconds_now();
      static uint32_t previousTime = 0;
      if((currentTime - previousTime) > DEBOUNCE)
        {
          freqTrackerRight += 1;
          //serial0_print_string("\nadd 1");
        }
        previousTime = currentTime;
    }

    if((freqTimeNow - freqTimeBefore) >= 2000)
    {
      if(freqTrackerLeft > freqTrackerRight)
      {
        // sprintf(serial_string, "\nFreq Tracker Left: %4u", freqTrackerLeft);
        // serial0_print_string(serial_string);
        frequency = freqTrackerLeft/2;
        freqTrackerLeft = 0;
        freqTrackerRight = 0;
        sprintf(serial_string,"\nLeft LDR: %3u, Right LDR: %3u, Frequency Left (HZ): %3u", leftLDR, rightLDR, frequency); //Format string
        serial0_print_string(serial_string);
        freqTimeBefore = freqTimeNow; 
      }
      else
      {
        // sprintf(serial_string, "\nFreq Tracker Right: %4u", freqTrackerRight);
        // serial0_print_string(serial_string);
        frequency = freqTrackerRight/2;
        freqTrackerRight = 0;
        freqTrackerLeft = 0;
        sprintf(serial_string,"\nLeft LDR: %3u, Right LDR: %3u, Frequency Right(HZ): %3u", leftLDR, rightLDR, frequency); //Format string
        serial0_print_string(serial_string);
        freqTimeBefore = freqTimeNow; 
      }
    }

   
    if(batteryVoltage < 7000)
    {
      PORTL |= (1<<PL2);
    }
    else
    {
      PORTL &= ~(1<<PL2);
    }

    if(gripperState)
    {
      OCR3A = 1700;
    }
    else
    {
      OCR3A = 1000;
    }

    

    
    // receiving section
    if(serial2_available()) //Returns true if new data available on serial buffer
    {
      //Function takes the array to return data to and the number of bytes to be read.
      serial2_get_data(recievedData,6);
      // sprintf(serial_string,"\nX DATA: Data 1: %3u, Data2: %3u", recievedData[0],recievedData[1]); //Format string
	    // serial0_print_string(serial_string); //Print string to usb serial
      // sprintf(serial_string,"\nY DATA: Data 3: %3u, Data4: %3u", recievedData[2],recievedData[3]); //Format string
      // serial0_print_string(serial_string);
      combinedDatax = (uint16_t)recievedData[0] << 8 | recievedData[1];
      combinedDatay = (uint16_t)recievedData[2] << 8 | recievedData[3];
      // combinedDataz = (uint16_t)recievedData[4] << 8 | recievedData[5];

      // sprintf(serial_string,"\nData 4: %3u, Data5: %3u", recievedData[4],recievedData[5]); //Format string
      // serial0_print_string(serial_string);

      
      if(recievedData[4] == 1)
      {
        robotState = 1;
      }
      else
      {
        robotState = 0;
      }
      // sprintf(serial_string,"\nrec 4: %3u", recievedData[4]); //Format string
      // serial0_print_string(serial_string);
       if(recievedData[5] == 1)
      {
        gripperState = 1;
      }
      else
      {
        gripperState = 0;
      }
      // sprintf(serial_string,"\nZ rec 5: %3u", recievedData[5]); //Format string
      // serial0_print_string(serial_string);
      
      if(gripperState)
      {
        OCR3A = 1700;
      }
      else
      {
        OCR3A = 1000;
      }

      // sprintf(serial_string,"\ncomp: %3u", compValue); //Format string
      // serial0_print_string(serial_string);

      fc = (int16_t)combinedDatay - 512;
      //sprintf(serial_string,"\nfc: %3i", fc);
      //serial0_print_string(serial_string);
      rc = (int16_t)combinedDatax - 512;
      //sprintf(serial_string,"\nrc: %3i", rc);
      //serial0_print_string(serial_string);
      leftMotor = fc + rc; 
      rightMotor = fc - rc;
	  }
    
    //sending section
    if((current_ms-last_send_ms) >= 100) //sending rate controlled here
    {
      // serial0_print_string("\nSent");
      //Arbitrary process to update databytes
      databyte1 = (batteryVoltage >> 8); // MSB
      if(databyte1 > 253)
      {
        databyte1 = 253;
      }
      databyte2 = batteryVoltage;        // LSB
      if(databyte2 > 253)
      {
        databyte2 = 253;
      }
      databyte3 = frequency;
      if(databyte3 > 253)
      {
        databyte3 = 253;
      }
      databyte4 = frontSensor;
      if(databyte4 > 253)
      {
        databyte4 = 253;
      }
      databyte5 = leftSensor;
      if(databyte5 > 253)
      {
        databyte5 = 253;
      } 
      databyte6 = rightSensor;
      if(databyte6 > 253)
      {
        databyte6 = 253;
      }           


      // sprintf(serial_string,"\nDatabye1: %3u, Databyte2: %3u, Databyte3: %3u", databyte1, databyte2, databyte3); //Format string
      // serial0_print_string(serial_string);
      //Function takes the number of bytes to send followed by the databytes as arguments
      serial2_write_bytes(6, databyte1, databyte2, databyte3, databyte4, databyte5, databyte6); 
      last_send_ms = current_ms;
    }
  
    // sprintf(serial_string,"\nrm: %3i", rightMotor);
    // serial0_print_string(serial_string);
    // sprintf(serial_string,"\nlm: %3i", leftMotor);
    // serial0_print_string(serial_string);
    

    if(!robotState) // if robotState = 0: remote control
    {
      leftMotorSpeed = (int32_t)abs(leftMotor)*20000/1023; //lm speed from magnitude of lm range -512 to 512 but abolute value
      if(leftMotorSpeed > 10000)
      {
        leftMotorSpeed = 10000;
      }
      
      // sprintf(serial_string,"\nLeft motor speed: %3u", leftMotorSpeed); //Format string
      // serial0_print_string(serial_string);
      OCR1B = leftMotorSpeed;
      
      rightMotorSpeed = (int32_t)abs(rightMotor)*20000/1023; //rm speed from magnitude of rm range 1- 1023
      if(rightMotorSpeed > 10000)
      {
        rightMotorSpeed = 10000;
      }
    
      // sprintf(serial_string,"\nRight motor speed: %3u", rightMotorSpeed); 
      // serial0_print_string(serial_string);
      OCR1A = rightMotorSpeed;
      
      if(leftMotor>=20) //if lm is positive
      {
        //set direction forwards
        PORTL |= (1<<PL0);
        PORTL &= ~(1<<PL1);


      }
      else if(leftMotor<=-20)
      {
        //set direction reverse
        PORTL &= ~(1<<PL0);
        PORTL |= (1<<PL1);
      }
      else
      {
        // motors off / deadzone
        PORTL &= ~(1<<PL0);
        PORTL &= ~(1<<PL1);
      }

      if(rightMotor>=20) //if rm is positive
      {
        //set direction forwards
        PORTA |= (1<<PA0);
        PORTA &= ~(1<<PA1);
      }
      else if(rightMotor<=-20)
      {
        PORTA &= ~(1<<PA0);
        PORTA |= (1<<PA1);
      }
      else
      {
        // motors off / deadzone
        PORTA &= ~(1<<PA0);
        PORTA &= ~(1<<PA1);
      }
    }
    else if(robotState && gripperState)
    {
      OCR1A = 10000;
      OCR1B = 10000;
      // sprintf(serial_string,"\nFrequency (HZ): %3u", frequency); //Format string
      // serial0_print_string(serial_string);
      if (frontSensor < FRONT_THRESHOLD)
      {
        //stop right motor
        PORTA &= ~(1<<PA0);
        PORTA &= ~(1<<PA1);
        
        //stop left motor
        PORTL &= ~(1<<PL0);
        PORTL &= ~(1<<PL1);
      }
 
      else if(rightLDR < leftLDR)
      {
        turnRight(5);
        goForward();
        //_delay_ms(10);
      }
     else if(leftLDR < rightLDR)
      {
        turnLeft(5);
        goForward();
        //_delay_ms(10);
      }
      else
      {
        goForward();
        //_delay_ms(500);
      }
    }
    else  // autonomous
    {
      OCR1A = 5000;
      OCR1B = 5000;
      if(frontSensor < FRONT_THRESHOLD)
      {
        OCR1A = 0;  // stop right motor
        OCR1B = 0;  // stop left motor

        if(leftSensor <= SIDE_THRESHOLD && rightSensor >= SIDE_THRESHOLD)
        {
          turnRight(100);  // turn 90 degrees right
          //serial0_print_string("\nturning right");
        }
        else if(rightSensor <= SIDE_THRESHOLD && leftSensor >= SIDE_THRESHOLD)
        {
          turnLeft(100);   // turn 90 degrees left
          //serial0_print_string("\nturning left");
        }
        else
        {
          if(rightSensor > leftSensor)
          {
            turnRight(100);
            //serial0_print_string("\nturning right r > l");
          }
          else
          {
            turnLeft(100);
            //serial0_print_string("\nturning left l > r");
          }
        }
      }
      else if(leftSensor < SIDE_THRESHOLD)
      {
        turnRight(30);
        //serial0_print_string("\nadjust right");
      }
      else if(rightSensor < SIDE_THRESHOLD)
      {
        turnLeft(30);
        //serial0_print_string("\nadjust left");
      }
      
      else
      {
        goForward();
        //serial0_print_string("\nFORWARD");
      }
    }
  }return(1);
}//end main 

uint16_t getShortDistance(uint16_t adcInput) // short
{
  float xValue;
  float distance;
  float voltage;
  
  voltage = ((adcInput * 5.0) / 1023.0);
  
  xValue = (voltage - 0.01038) / 11.724;
      // Avoid division by zero
      if (xValue != 0) 
      {
        distance = ((1/xValue) - 0.42) * 10;
      } 
      else 
      {
        distance = 0; // Handle error or set to some default value
      }

  return (uint16_t)(distance);
}

uint16_t getLongDistance(uint16_t adcInput) // long
{
  float xValue;
  float distance;
  float voltage;
  
  voltage = ((adcInput * 5.0) / 1023.0);
  
  xValue = (voltage - 0.356) / 18.89;
  
  distance = (1/xValue) * 10;

  return (uint16_t)(distance);
}

void turnRight(uint16_t turnTime)
{
  OCR1A = 6000;
  OCR1B = 6000;
  //stop right motor
  PORTA &= ~(1<<PA0);
  PORTA &= ~(1<<PA1);
  
  //stop left motor
  PORTL &= ~(1<<PL0);
  PORTL &= ~(1<<PL1);

  _delay_ms(1);
  
  //left motor forwards
  PORTL |= (1<<PL0);
  PORTL &= ~(1<<PL1);
  
  //right motor backwards
  PORTA &= ~(1<<PA0);
  PORTA |= (1<<PA1);
  
  //_delay_ms(1000);
  
  for(int i = 0; i <= turnTime; i++)
  {
    _delay_ms(1);
  }
  
  //stop right motor
  PORTA &= ~(1<<PA0);
  PORTA &= ~(1<<PA1);
  
  //stop left motor
  PORTL &= ~(1<<PL0);
  PORTL &= ~(1<<PL1);
}

void turnLeft(uint16_t turnTime)
{
  OCR1A = 6000;
  OCR1B = 6000;
  //stop right motor
  PORTA &= ~(1<<PA0);
  PORTA &= ~(1<<PA1);
  
  //stop left motor
  PORTL &= ~(1<<PL0);
  PORTL &= ~(1<<PL1);

  _delay_ms(1);

  //left motor backwards
  PORTL &= ~(1<<PL0);
  PORTL |= (1<<PL1);

  //right motor forwards
  PORTA |= (1<<PA0);
  PORTA &= ~(1<<PA1);
  
  for(int i = 0; i <= turnTime; i++)
  {
    _delay_ms(1);
  }
  
  //stop right motor
  PORTA &= ~(1<<PA0);
  PORTA &= ~(1<<PA1);
  
  //stop left motor
  PORTL &= ~(1<<PL0);
  PORTL &= ~(1<<PL1);
}

void goForward()
{
  OCR1A = 5000;
  OCR1B = 5000;  
  //left motor forwards
  PORTL |= (1<<PL0);
  PORTL &= ~(1<<PL1);

  //right motor forwards
  PORTA |= (1<<PA0);
  PORTA &= ~(1<<PA1);
}