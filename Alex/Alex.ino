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

#define F_CPU 16000000 //cpu clock speed

#define READ_BUFFER_SIZE 140
#define WRITE_BUFFER_SIZE 140
// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      180.0

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5 // Left forward pin
#define LR                  6 // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

//PI, for calculating turn circumference
#define PI                      3.141592654

// Alex's length and breadth in cm
#define ALEX_BREADTH   9.8

// Alex's turning circumference, calculated once
float alexCirc = 0.0;

// UDRIE mask. Use this to enable/disable the UDRE interrupt
#define UDRIEMASK           0b00100000

// set params for ultrasound
#define USDELAY      50
#define USTHRESHOLD  10
unsigned long ustime = millis();


/*
 *    Alex's State Variables
 */


// Serial buffers
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

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftForwardRevs;
volatile unsigned long rightForwardRevs;
volatile unsigned long leftReverseRevs;
volatile unsigned long rightReverseRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long leftusdistance;
unsigned long rightusdistance;


/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;
    len = readSerial(buffer);

    if(len == 0){
      return PACKET_INCOMPLETE;
    }
    else
      return deserialize(buffer, len, packet);
    
}

void delaymicros(int delaytime){
  long long timenow = micros();
  while (micros() - timenow < delaytime){   
  }
}

//to obtain distance between left ultrasonic sensor
//and nearest obstacles at regular time intervals
void updateleftus(){
  PORTD &= ~(1 << 7);
  long long timenow = micros();
  delaymicros(2);
  //delayMicroseconds(2);
  PORTD |= (1 << 7);
  delaymicros(10);
  //delayMicroseconds(10);
  PORTD &= ~(1 << 7);
  long duration = pulseIn(8, HIGH);
  leftusdistance = (duration) * 0.034 / 2.0;
}

//to obtain distance between right ultrasonic sensor
//and nearest obstacles at regular time intervals
void updaterightus(){
  PORTB &= ~(1 << 3);
  delaymicros(2);
  //delayMicroseconds(2);
  PORTB |= (1 << 3);
  delaymicros(10);
  //delayMicroseconds(10);
  PORTB &= ~(1 << 3);
  long duration = pulseIn(12, HIGH);
  rightusdistance = (duration) * 0.034 / 2.0;
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
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

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  clearReadBuffer();
  clearWriteBuffer();
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= (~(1 << DDD3) & ~(1 << DDD2));
  PORTD |= (1 << PORTD3) | (1 << PORTD2);
}

void setupUltrasound()
{
  //make pin 7 and pin 11 output, then pin 8 and pin 12 input using bare-metal and set them as pulldown
  DDRD |= (1 << DDD7);
  DDRB |= (1 << DDB3);
  DDRB &= (~(1 << DDB4) & ~(1 << DDB0));
  PORTB &= (~(1 << PORTB4) & ~(1 << PORTB0));
}

void setupLED()
{
  //make A0-A3 output using bare-metal and set them as digital 0 first
  DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
  PORTC &= (~(1 << DDC0) & ~(1 << DDC1) & ~(1 << DDC2) & ~(1 << DDC3));
}

// Functions to be called by INT0 and INT1 ISRs.
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
  // leftTicks++;
  // leftRevs = leftTicks / COUNTS_PER_REV;
  // Serial.print("LEFT: ");
  // Serial.println(leftTicks);
  // Serial.print("LEFT REVS: ");
  // Serial.println(leftRevs);
}

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
  // Serial.print("RIGHT: ");
  // Serial.println(rightTicks);
  // rightRevs = rightTicks / COUNTS_PER_REV;
  // Serial.print("RIGHT REVS: ");
  // Serial.println(rightRevs);
}



// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli(); //disable the interrupts
  EICRA |= 0b00001010; //FALLING edge
  EIMSK |= 0b00000011; //Switch on interrupts for INT0 and INT1
  sei(); //enable the interrupts
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
 float baudRate = 9600;
 unsigned int b = (unsigned int) ((float)F_CPU / (16.0 * baudRate)) - 1;
 
 UCSR0C = 0b00000110;//controls size set for 8N1
 UBRR0L = b;
 UBRR0H = (b >> 8);
 UCSR0A = 0;
  // Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
 UCSR0B = 0b10011000;
}


TBufferStatus pushToBuffer(TCircBuffer *circBuffer, char data) {
  if (circBuffer->len >= circBuffer->max_size) return BUFFER_FULL; // ! If buffer is full fail silently and lose data
  
  circBuffer->buffer[circBuffer->back] = data; // Get received data
  circBuffer->len++;
  circBuffer->back = (circBuffer->back + 1) % circBuffer->max_size;
    
  return BUFFER_OK;
}

TBufferStatus popFromBuffer(TCircBuffer *circBuffer, char *data) {
    if (circBuffer->len <= 0) return BUFFER_EMPTY;

    *data = circBuffer->buffer[circBuffer->front];
    circBuffer->len--;
    circBuffer->front = (circBuffer->front + 1) % circBuffer->max_size;

    return BUFFER_OK;
}

ISR(USART_RX_vect) {
  char data = UDR0;

  pushToBuffer(&readBuffer, data);
}


// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

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


// Write to the serial port. Replaced later with
// bare-metal code


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
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
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

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist;
  }
  else{
    deltaDist=9999999;
  }
  newDist=forwardDist + deltaDist;
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.
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

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist;
  }
  else{
    deltaDist=9999999;
  }
  newDist=reverseDist + deltaDist;
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9. 
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

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long)(((ang/360.0) * alexCirc)/(WHEEL_CIRC/COUNTS_PER_REV));
  //unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) /( 360.0 * WHEEL_CIRC));
  return ticks;
}



// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  int val = pwmVal(speed);
  dir = LEFT;
  //speed = (int)((speed/100.0) * 255.0);
  
  OCR1A = 0;
  OCR0A = (int)( 0.95 * val);
  OCR0B = 0;
  OCR1B = val;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
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

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  OCR1A = 0;
  OCR0B = 0;
  OCR0A = 0;  
  OCR1B = 0;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
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

void clearReadBuffer() {
  readBuffer.len = 0;
  readBuffer.back = 0;
  readBuffer.front = 0;
}

void clearWriteBuffer() {
  writeBuffer.len = 0;
  writeBuffer.back = 0;
  writeBuffer.front = 0;
}

// Clears one particular counter
void clearOneCounter()
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
  clearReadBuffer();
  clearWriteBuffer();
  readBuffer.max_size = READ_BUFFER_SIZE;
  writeBuffer.max_size = WRITE_BUFFER_SIZE;
  alexCirc = PI * ALEX_BREADTH;
}

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
    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

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

void setup() {
  // put your setup code here, to run once:
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
  //waitForHello();
}

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

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
// Uncomment the code below for Week 9 Studio 2
  if (millis() - ustime >= USDELAY){
    updateleftus();
    updaterightus();
    ustime = millis();
  }
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
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
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
