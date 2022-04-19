#include <serialize.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * Alex's configuration constants
 */
volatile TDirection dir = STOP;
#define F_CPU 16000000 //CPU internal clock speed
#define READ_BUFFER_SIZE 140  // Read Buffer Size
#define WRITE_BUFFER_SIZE 140  // Write Buffer Size

// For calculation of forward/backward distance traveled by taking revs * WHEEL_CIRC
#define COUNTS_PER_REV 180.0  // Number of ticks per revolution from the wheel encoder.
#define WHEEL_CIRC 20.4  // Wheel circumference, WHEEL_CIRC in cm.
#define PI  3.141592654  //PI, for calculating turn circumference
#define ALEX_BREADTH 9.8  // Alex's breadth in cm
float alexCirc = 0.0;  // Alex's turning circumference, calculated once

// Motor control pins. 
#define LF  5 // Left forward pin
#define LR  6 // Left reverse pin
#define RF  10  // Right forward pin
#define RR  9  // Right reverse pin

#define UDRIEMASK 0b00100000  // UDRIE mask. Use this to enable/disable the UDRE interrupt

// Set params for ultrasound
#define USDELAY      50  // Time taken for delay
#define USTHRESHOLD  10  // Distance in cm to trigger LEDs to light up
unsigned long ustime = millis();


/*
 * Alex's State Variables
 */
// Serial Circular buffers
typedef struct
{
  int len;
  int front;
  int back;
  int max_size;
  char buffer[WRITE_BUFFER_SIZE];
} TCircBuffer;

TCircBuffer writeBuffer;
TCircBuffer readBuffer;

unsigned long prev = millis(); // Watch time for serial communication delay

// Store the number of ticks in different directions from Alex's left and right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the number of revolutions on Alex's left and right wheels
volatile unsigned long leftForwardRevs;
volatile unsigned long rightForwardRevs;
volatile unsigned long leftReverseRevs;
volatile unsigned long rightReverseRevs;

// Store the distance travelled in the forward and backward direction
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Store the distance of the nearest obstacle from Alex using distance measured by the ultrasonic sensor
unsigned long leftusdistance;
unsigned long rightusdistance;


/*
 * @PARAM [packet] To store the deserialised data
 * Alex Communication Routines.
 * Reads in data from the serial port and deserializes it. Returns deserialized data in "packet".
 */
TResult readPacket(TPacket *packet)
{
    char buffer[PACKET_SIZE];
    int len;
    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
}

/*
 * @PARAM [delaytime] Time duration to delay in microseconds
 * For time delay of a specific duration
 */
void delaymicros(int delaytime){
  long long timenow = micros();
  while (micros() - timenow < delaytime){   
  }
}


/*
 * To obtain distance between left ultrasonic sensor 
 * and nearest obstacles at regular time intervals
 */
void updateleftus(){
  PORTD &= ~(1 << 7);
  delaymicros(2);
  PORTD |= (1 << 7);
  delaymicros(10);
  PORTD &= ~(1 << 7);
  long duration = pulseIn(8, HIGH);
  leftusdistance = (duration) * 0.034 / 2.0;
}

/*
 * To obtain distance between right ultrasonic sensor 
 * and nearest obstacles at regular time intervals
 */
void updaterightus(){
  PORTB &= ~(1 << 3);
  delaymicros(2);
  PORTB |= (1 << 3);
  delaymicros(10);
  PORTB &= ~(1 << 3);
  long duration = pulseIn(12, HIGH);
  rightusdistance = (duration) * 0.034 / 2.0;
}


/*
 * To send a packet containing key information regarding Alex's motors' ticks,
 * forward/reverse distance and nearest obstacle distance.
 * Params array is used to store this information,
 * and set the packetType and command files accordingly.
 * sendResponse function is used to send the packet out.
 */
void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = leftusdistance;
  statusPacket.params[11] = rightusdistance;
  
  sendResponse(&statusPacket); 

}

/*
 * @PARAM [*message] Stores an array of char that contains the message to be sent across
 * Sends text messages back to the Pi. Useful for debugging.
 */
void sendMessage(const char *message)
{ 
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

/*
 * @PARAM [*format] Contains the array of character on the text to be sent
 * @PARAM [...] %[flags][width][.precision][length]specifier
 * Sends a message packet to the Pi
 */
void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

/*
 * Tell the Pi that it sent us a bad packet with a bad magic number.
 * Clears both the read and write buffer.
 */
void sendBadPacket()
{
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badPacket);
}

/*
 * Tell the Pi that it sent us a packet with a bad checksum.
 * Clears both the read and write buffer.
 */
void sendBadChecksum()
{
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badChecksum);  
}

/*
 * Tell the Pi that we don't understand its command sent to us.
 * Clears both the read and write buffer.
 */
void sendBadCommand()
{
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badCommand);
}

/*
 * Tell the Pi that the response sent to it cannot be understood.
 * Clears both the read and write buffer.
 */
void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badResponse);
}

/*
 * Tell the Pi that the command sent to it is understood.
 */
void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

/*
 * @PARAMS [*packet] Contains information of the packet data to be sent over. 
 * Takes a packet, serializes it then sends it out over the serial port.
 */ 
void sendResponse(TPacket *packet)
{
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and pullup resistors on pins 2 and 3 (PD2 and PD3).
 * Bits 2 and 3 in DDRD to 0 to make them inputs.
 */
void enablePullups()
{
  DDRD &= (~(1 << DDD3) & ~(1 << DDD2));
  PORTD |= (1 << PORTD3) | (1 << PORTD2);
}

/*
 * Setup code for ultrasonic sensor.
 * Pins 7 and 11 are set as outputs (PD7 and PB3).
 * Pins 8 and 12 are set as inputs and as pulldown (PB0 and PB4).
 */
void setupUltrasound()
{
  //make pin 7 and pin 11 output, then pin 8 and pin 12 input using bare-metal and set them as pulldown
  DDRD |= (1 << DDD7);
  DDRB |= (1 << DDB3);
  DDRB &= (~(1 << DDB4) & ~(1 << DDB0));
  PORTB &= (~(1 << PORTB4) & ~(1 << PORTB0));
}

/*
 * Setup code for the LEDs
 * A0-A3 are set as output and set to digital 0 first (turned off).
 */
void setupLED()
{
  DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
  PORTC &= (~(1 << DDC0) & ~(1 << DDC1) & ~(1 << DDC2) & ~(1 << DDC3));
}

/*
 * Function to be called by INT0 ISR.
 * Updates the number of ticks in the left motor according to the direction.
 */
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == REVERSE) {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

/*
 * Function to be called by INT1 ISR.
 * Updates the number of ticks in the right motor according to the direction.
 */
void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == REVERSE) {
    rightReverseTicks++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
}

/*
 * Set up the external interrupt pins INT0 and INT1 for falling edge triggered
 * Enable the INT0 and INT1 interrupts.
 */
void setupEINT()
{
  cli(); //disable the interrupts
  EICRA |= 0b00001010; //FALLING edge
  EIMSK |= 0b00000011; //Switch on interrupts for INT0 and INT1
  sei(); //enable the interrupts
}

/*
 * External interrupt ISR function.
 * INT0 ISR should call leftISR.
 */
ISR(INT0_vect)
{
  leftISR();
}

/*
 * External interrupt ISR function.
 * INT1 ISR should call rightISR.
 */
ISR(INT1_vect)
{
  rightISR();
}


/*
 * Setup code for serial communications in bare metal
 */
void setupSerial()
{
 float baudRate = 9600;
 unsigned int b = (unsigned int) ((float)F_CPU / (16.0 * baudRate)) - 1;
 
 UCSR0C = 0b00000110;//controls size set for 8N1
 UBRR0L = b;
 UBRR0H = (b >> 8);
 UCSR0A = 0;
}

/*
 * Start code for serial communications in bare metal
 */
void startSerial()
{
 UCSR0B = 0b10011000;
}

/*
 * @PARAM [*circBuffer] Circular buffer that stores and transfers data
 * @PARAM [data] A character in the data
 * Push char data to the circular buffer.
 * Returns BUFFER_FULL if the buffer is full and BUFFER_OK if buffer still has space
 */
TBufferStatus pushToBuffer(TCircBuffer *circBuffer, char data) {
  if (circBuffer->len >= circBuffer->max_size) return BUFFER_FULL; // If buffer is full, fail silently and lose data
  
  circBuffer->buffer[circBuffer->back] = data; // Get received data
  circBuffer->len++;
  circBuffer->back = (circBuffer->back + 1) % circBuffer->max_size;
    
  return BUFFER_OK;
}

/*
 * @PARAM [*circBuffer] Circular buffer that stores and transfers data
 * @PARAM [data] A character in the data
 * Once data is read, pop data off the circular buffer so that next data can be read in
 * Returns BUFFER_EMPTY when the buffer is empty and BUFFER_OK if the buffer still has data
 */
TBufferStatus popFromBuffer(TCircBuffer *circBuffer, char *data) {
    if (circBuffer->len <= 0) return BUFFER_EMPTY;

    *data = circBuffer->buffer[circBuffer->front];
    circBuffer->len--;
    circBuffer->front = (circBuffer->front + 1) % circBuffer->max_size;

    return BUFFER_OK;
}

/*
 * Interrupt that is called when a data is received using serial communication
 */
ISR(USART_RX_vect) {
  char data = UDR0;

  pushToBuffer(&readBuffer, data);
}

/*
 * @PARAM [*buffer] An array that stores the data
 * Starts reading the buffer from the serial port when circular buffer size exceeds the size of TPacket. 
 * Keeps reading from circular buffer until BUFFER_EMPTY
 * Returns the count of how many bytes were read from the Serial port
 */
int readSerial(char *buffer)
{
  int count = 0;

  // while (Serial.available()){
  //   buffer[count++] = Serial.read();
  // }

  TBufferStatus status;
  if (readBuffer.len >= sizeof(TPacket)){
    do {
      status = popFromBuffer(&readBuffer, &(buffer[count]));

      if (status == BUFFER_OK) {
        count++;
        prev = millis();
        while (millis() - prev < 1); //ensure that there is a delay between popping from buffer
      }
    } while (status == BUFFER_OK);
  }
  
  return count;
}

/*
 * Interrupt that is called when a data is transmitted using serial communication
 * If BUFFER_OK, write data to Serial port.
 * If BUFFER_EMPTY, disable the UDRE interrupt as there is no more data to send.
 */
ISR(USART_UDRE_vect) {
  char data;
  TBufferStatus status = popFromBuffer(&writeBuffer, &data);

  if (status == BUFFER_OK){
    UDR0 = data; // Write data to to UDR0
  }
  else if (status == BUFFER_EMPTY) {
    UCSR0B &= ~UDRIEMASK; // Disable UDRE interrupt
    //writing = false;
  }
}


/*
 * @PARAM [*buffer] An array that stores the data
 * @PARAM [len] Total length of buffer
 * fills up the writeBuffer with the data we want to send,
 * then writes to UDR0 with the first data to get the ball rolling and
 * turns on the UDRE interrupt to start the continuous sending of data.
 */
void writeSerial(const char *buffer, int len) {
  // Serial.write(buffer,len);
  
  TBufferStatus status = BUFFER_OK;

  clearWriteBuffer();

  // Copy data to writeBuffer
  for (int i = 1; i < len && status == BUFFER_OK; i++) {
    status = pushToBuffer(&writeBuffer, buffer[i]);
  }

  //writing = true;

  // Get the ball rolling
  UDR0 = buffer[0];

  // Enable UDRE interrupt below
  UCSR0B |= UDRIEMASK;
  
}

/*
 * Setting up Alex's motors in baremetal
 */
void setupMotors()
{
  /* Our motor set up is:  
    LF - PIN 5 - OC0B
    LR - PIN 6 - OC0A
    RF - PIN 10 -  OC1B
    RR - PIN 9 - OC1A
   */
  DDRD |= (1 << 6 | 1 << 5); //output PIN5,6
  DDRB |= (1 << 1 | 1 << 2); //output PIN9,10
  TCNT0 = 0;
  TCNT1 = 0;
  TCCR0A = 0b10100001;
  TCCR1A = 0b10100001;
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0; 
}

/*
 * Start the left and right motors
 */
void startMotors()
{
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}

/*
 * @PARAM [speed] Numerical percentage of the motor speed
 * Convert percentages to PWM values
 */
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

/*
 * @PARAM [dist] Numerical distance in cm in which the 
 * @PARAM [speed] Numerical percentage of the motor speed
 * Causes the motor to move forward by a specific distance and speed.
 * Specifying a distance of 0 means Alex will continue moving forward indefinitely.
 */
void forward(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist;
  }
  else{
    deltaDist=9999999;
  }
  newDist=forwardDist + deltaDist;
  int val = pwmVal(speed);
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  dir = FORWARD;

  OCR1A = 0;
  OCR0A = 0;
  OCR0B = (int)( 0.95 * val);  
  OCR1B = val;
}

/*
 * @PARAM [dist] Numerical distance in cm in which the 
 * @PARAM [speed] Numerical percentage of the motor speed
 * Causes the motor to reverse by a specific distance and speed.
 * Specifying a distance of 0 means Alex will continue reversing indefinitely.
 */
void reverse(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist;
  }
  else{
    deltaDist=9999999;
  }
  newDist=reverseDist + deltaDist;
  int val = pwmVal(speed);
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  dir = REVERSE;
  OCR1A = val;
  OCR0A = (int)( 0.95 * val);
  OCR0B = 0;  
  OCR1B = 0;
}

/*
 * @PARAM [ang] Angle at which Alex should turn
 * Returns the number of ticks caused by Alex's turning
 */
unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long)(((ang/360.0) * alexCirc)/(WHEEL_CIRC/COUNTS_PER_REV));
  //unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) /( 360.0 * WHEEL_CIRC));
  return ticks;
}

/*
 * @PARAM [ang] Angle at which Alex should turn
 * @PARAM [speed] Numerical percentage of the motor speed
 * Causes the motor to turn left by a specific angle and speed.
 * Specifying a distance of 0 means Alex will continue to turn left indefinitely.
 */
void left(float ang, float speed)
{
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  int val = pwmVal(speed);
  dir = LEFT;
  //speed = (int)((speed/100.0) * 255.0);
  
  OCR1A = 0;
  OCR0A = (int)(0.95 * val);
  OCR0B = 0;
  OCR1B = val;
}

/*
 * @PARAM [ang] Angle at which Alex should turn
 * @PARAM [speed] Numerical percentage of the motor speed
 * Causes the motor to turn left by a specific angle and speed.
 * Specifying a distance of 0 means Alex will continue to turn right indefinitely.
 */
void right(float ang, float speed)
{
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  int val = pwmVal(speed);
  dir = RIGHT;
  //speed = (int)((speed/100.0) * 255.0);
  OCR1A = val;
  OCR0A = 0;
  OCR0B = (int)( 0.95 * val);  
  OCR1B = 0;
}

// Stopping Alex in baremetal.
void stop()
{
  dir = STOP;
  OCR1A = 0;
  OCR0B = 0;
  OCR0A = 0;  
  OCR1B = 0;
}

/*
 * To clear all counters, which includes the ticks and distance variables
 */
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

/*
 * To clear all read buffers.
 */
void clearReadBuffer() {
  readBuffer.len = 0;
  readBuffer.back = 0;
  readBuffer.front = 0;
}

/*
 * To clear all write buffers.
 */
void clearWriteBuffer() {
  writeBuffer.len = 0;
  writeBuffer.back = 0;
  writeBuffer.front = 0;
}

/*
 * To clear one counter.
 */
void clearOneCounter()
{
  clearCounters();
}

/*
 * To intialize Alex's internal states
 */
void initializeState()
{
  clearCounters();
  clearReadBuffer();
  clearWriteBuffer();
  readBuffer.max_size = READ_BUFFER_SIZE;
  writeBuffer.max_size = WRITE_BUFFER_SIZE;
  alexCirc = PI * ALEX_BREADTH;
}

/*
 * @PARAM [*command] The packet command that was sent from Pi to Arduino
 * Receives a packet command from Pi and executes the command depending on
 * what command type it is.
 */
void handleCommand(TPacket *command)
//can change initial name
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter();
      break;
        
    default:
      sendBadCommand();
  }
}

/*
 * Handshake between Pi and Arduino
 * Run once during setup to ensure that both Pi and Arduino are connected
 */
void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

/*
 * Setup code for Arduino.
 * Setup and start serial, motors, interrupt, LEDs, ultrasonic pins.
 */
void setup() {
  // put setup code here, to run once:
  cli();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  setupEINT();
  setupLED();
  enablePullups();
  setupUltrasound();
  initializeState();
  sei();
  waitForHello();
}

/*
 * @PARAM [*packet] The packet that was sent from Pi to Arduino
 * Receives a packet from Pi and splits into different cases to handle different types of packets.
 * If the packet is of type command, handleCommand function will be called.
 */
void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // when time to update ultrasound, it will be updated
  if (millis() - ustime >= USDELAY){
    updateleftus();
    updaterightus();
    ustime = millis();
  }
  // If the ultrasonic sensor receives a distance of less than the threshold value,
  // the red LEDs will light up according to the side of the ultrasonic sensor that receives the distance less than USTHRESHOLD.
  // If Alex starts moving, the green LEDs would light up.
  if (leftusdistance < USTHRESHOLD){
    PORTC |= (1 << PORTC3);
  } else {
    PORTC &= ~(1 << PORTC3);
  }
  if (rightusdistance < USTHRESHOLD){
    PORTC |= (1 << PORTC0);
  } else {
    PORTC &= ~(1 << PORTC0);
  }
  if (dir == STOP){
    PORTC &= (~(1 << PORTC1) & ~(1 << PORTC2)); 
  } else {
    PORTC |= (1 << PORTC1) | (1 << PORTC2);
  }

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket); // reads the packet received from Pi
  
  // Handles the packet received from Pi accordingly
  // If the packet has no error, handlePacket function will be called.
  // If the packet has error (bad magic number or bad checksum), sendBadPacket and sendBadChecksum will be called respectively.
  if(result == PACKET_OK){
    handlePacket(&recvPacket);
  }
  else if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
  else if(result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    } 

  // To reset the deltaDist and newDist variables whenever an 'error' occurs,
  // which is when the current distance is greater than the newDist and deltaDist > 0.
  if(deltaDist > 0)
  {
    if(dir==FORWARD)
    {
      if(forwardDist > newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else if(dir == REVERSE)
    {
      if(reverseDist > newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else if(dir == STOP)
    {
      deltaDist=0;
      newDist=0;
      stop();
    }
  }

  // To reset the deltaTicks and targetTicks variables whenever an 'error' occurs,
  // which is when the current number of ticks is greater than the targetTicks and deltaTicks > 0.
  if(deltaTicks > 0)
  {
    if(dir == LEFT)
    {
      if(leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();     
      }
    }
    else if(dir == RIGHT)
    {
      if(rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();  
      }
    }
    else if(dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  } 
}
