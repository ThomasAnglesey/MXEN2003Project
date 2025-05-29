//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#define DEBOUNCE 200

//static function prototypes, functions only called in this file

volatile uint16_t robotState = 0;
volatile uint16_t gripperState = 0;

int main(void)
{
  cli();

  DDRD = 0;                                   // PORTD as input
  PORTD |= (1<<PD1)|(1<<PD0);                 // pullup
  PORTD |= (1<<PD2);
  EIMSK |= (1<<INT1)|(1<<INT0);               // enable int0
  EICRA |= (1<<ISC11)|(1<<ISC01);             // falling edge

  serial0_init();
  serial2_init();
  milliseconds_init();
  adc_init();
  lcd_init();
  
  uint32_t current_ms;
  uint32_t last_send_ms;
  uint8_t databyte1 = 0;
  uint8_t databyte2 = 0;
  uint8_t databyte3 = 0;
  uint8_t databyte4 = 0;
  uint8_t databyte5 = 0;
  uint8_t databyte6 = 0;
  uint16_t joystickPositionx;
  uint16_t joystickPositiony;
  //uint16_t joystickPositionz;
  uint8_t recievedData[6]; //recieved data array
  uint16_t combinedData;
  char voltageString[10];
  char serial_string[60] = {0}; // String used for printing to terminal
  uint32_t printTimeBefore;
  uint32_t printTimeAfter;

  sei();

  while(1)//main loop
  {
    //main loop
    current_ms = milliseconds_now();
    printTimeBefore = milliseconds_now();
    joystickPositionx = adc_read(1);
    joystickPositiony = adc_read(0);
    // sprintf(serial_string,"\nadc14: %3u", joystickPositionz); //Format string
    // serial0_print_string(serial_string); //Print string to usb serial

    if((printTimeBefore - printTimeAfter) >= 1000)
    {
      if(robotState)
      {
        serial0_print_string("\nAutonomous mode");
        sprintf(serial_string,"\nFront Sensor: %3umm, Left Sensor: %3umm, Right Sensor: %3umm", recievedData[3], recievedData[4],recievedData[5]); //Format string
        serial0_print_string(serial_string); //Print string to usb serial
      }
      else
      {
        serial0_print_string("\nRemote Control Mode");
      }
      if(gripperState)
      {
        serial0_print_string("\nGripper Closed");
      }
      else
      {
        serial0_print_string("\nGripper Open");
      }

      printTimeAfter = printTimeBefore;
    }

    //sending section
    if((current_ms-last_send_ms) >= 100) //sending rate controlled here
    {
      //Arbitrary process to update databytes
      databyte1 = (joystickPositionx >> 8); // MSB
      if(databyte1 > 253)
      {
        databyte1 = 253;
      }
      databyte2 = joystickPositionx;        // LSB
      if(databyte2 > 253)
      {
        databyte2 = 253;
      }
      databyte3 = (joystickPositiony >> 8);
      if(databyte3 > 253)
      {
        databyte3 = 253;
      }
      databyte4 = joystickPositiony; 
      if(databyte4 > 253)
      {
        databyte4 = 253;
      }
      if(robotState)
      {
        databyte5 = 1;
      }
      else
      {
        databyte5 = 0;
      }
      if(gripperState)
      {
        databyte6 = 1;
      }
      else
      {
        databyte6 = 0;
      }

      // sprintf(serial_string,"\nZ DATA: Data 4: %3u, Data5: %3u", databyte5,databyte6); //Format string
      // serial0_print_string(serial_string);

      //Function takes the number of bytes to send followed by the databytes as arguments
      serial2_write_bytes(6, databyte1, databyte2, databyte3, databyte4, databyte5, databyte6); 
      last_send_ms = current_ms;
    }
    
    // receiving section
    if(serial2_available()) //Returns true if new data available on serial buffer
    {
      //serial0_print_string("inloop");
      //Function takes the array to return data to and the number of bytes to be read.
      serial2_get_data(recievedData,6); 
      sprintf(serial_string,"\nData 1: %3umm, Data 2: %3umm, Data 3: %3umm", recievedData[0], recievedData[1],recievedData[2]); //Format string
      serial0_print_string(serial_string); //Print string to usb serial
      lcd_clrscr();
      lcd_home();
      combinedData = (uint16_t)recievedData[0] << 8 | recievedData[1];
      sprintf(voltageString, "Voltage: %4umV", combinedData);
      serial0_print_string(voltageString);
      sprintf(serial_string, "Frequency: %3uHz", recievedData[2]);
      lcd_puts(voltageString);
      lcd_goto(0x40);
      lcd_puts(serial_string);
    }
  }return(1);
}//end main 

ISR(INT1_vect)
{   
  uint32_t currentTime = milliseconds_now();
  static uint32_t previousTime = 0;
  if((currentTime - previousTime) > DEBOUNCE)
  {
    robotState = !robotState;
    serial0_print_string("robot changed");
  }
  previousTime = currentTime;
}
ISR(INT0_vect)
{   
  uint32_t currentTime = milliseconds_now();
  static uint32_t previousTime = 0;
  if((currentTime - previousTime) > DEBOUNCE)
  {
    gripperState = !gripperState;
    serial0_print_string("gripper changed");
  }
  previousTime = currentTime;
}