#include "make_tls_server.h"
#include "tls_common_lib.h"
#include "netconstants.h"
#include "constants.h"
#include "packet.h"
#include "serial.h"
#include "serialize.h"

//Port Name of our Arduino
#define PORT_NAME			"/dev/ttyACM0"

#define BAUD_RATE			B9600

// TLS Port Number
#define SERVER_PORT			5000


//Constants we defined for our filenames in order:
//Alex's private key, Alex's certificate, CA cerficiate name, Common name for our laptop

#define PORTNUM 5000
#define KEY_FNAME   "alex.key"
#define CERT_FNAME  "alex.crt"
#define CA_CERT_FNAME   "signing.pem"
#define CLIENT_NAME     "laptop.app.com"


// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN				129

// delay between commands
#define COMMAND_DELAY 1000*1000 //in microseconds

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.

static volatile int networkActive;

// This variable is used by sendNetworkData to send back responses
// to the TLS connection.  It is sent by handleNetworkData

static void *tls_conn = NULL;

pthread_t serThread;
bool serialActive = false;
/*

	Alex Serial Routines to the Arduino

	*/

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

/*
This function handles an error of PACKETTYPE error,
found in the handleUART packet function

@param packet Data packet sent of type TPacket

**/
void handleErrorResponse(TPacket *packet)
{
	printf("UART ERROR: %d\n", packet->command);
	char buffer[2];
	buffer[0] = NET_ERROR_PACKET;
	buffer[1] = packet->command;
	sendNetworkData(buffer, sizeof(buffer));
}

/*
This function handles an error of PACKETTYPE error,
found in the handleUARTPacket function

@param packet Data packet sent of type TPacket

**/
void handleMessage(TPacket *packet)
{
	char data[33];
	printf("UART MESSAGE PACKET: %s\n", packet->data);
	data[0] = NET_MESSAGE_PACKET;
	memcpy(&data[1], packet->data, sizeof(packet->data));
	sendNetworkData(data, sizeof(data));
}

/*
This function handles a response type of RESP_STATUS,
found in the handleResponse function

@param packet Data packet sent of type TPacket

**/
void handleStatus(TPacket *packet)
{
	char data[65];
	printf("UART STATUS PACKET\n");
	data[0] = NET_STATUS_PACKET;
	memcpy(&data[1], packet->params, sizeof(packet->params));
	sendNetworkData(data, sizeof(data));
}

/*
This function handles a type of PACKET_TYPE_RESPONSE,
found in the handleUARTPacket function

@param packet Data packet sent of type TPacket

**/
void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			char resp[2];
			printf("Command OK\n");
			resp[0] = NET_ERROR_PACKET;
			resp[1] = RESP_OK;
			sendNetworkData(resp, sizeof(resp));
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
		printf("Boo\n");
	}
}

/*
This function handles an incoming data packet sent over serial communication.
Different packetTypes are handled via different functions called with it

@param packet Data packet sent of type TPacket

**/
void handleUARTPacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

/*
This function sends outgoing data packet over serial communications
It first serializes the data, converting it to a stream of bytes

@param packet Data packet sent of type TPacket

**/
void uartSendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

/*
This function handles error when receiving data in the uartReceiveThread

@param error The type of error present of type TResult
**/
void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

/*
This function receives data sent over serial communications
It waits for the all the bytes to be completely sent over before handling the packet
**/
void *uartReceiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handleUARTPacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				} // result
		} // len > 0
	} // while
	pthread_exit(0);//added
}

/*

	Alex Network Routines

	*/

/*
This functions sets up the serial communication
Inside the start serial function, we provide the details of the 
port name, baud rate, data bits, parity bits, stop bits
**/
void setupSerial() {
	printf("Opening Serial Port\n");
	// Open the serial port
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
	printf("Done. Waiting 1 seconds for Arduino to reboot\n");
	sleep(1);
	serialActive = true;
	
	printf("DONE. Starting Serial Listener\n");
	pthread_create(&serThread, NULL, uartReceiveThread, NULL);
}

/*
This function sends the data over the TLS connection if our network is active

@param data Pointer to the beginning of the data bytes, passed into SSLWrite function
@param len The length of our data sent
**/
void sendNetworkData(const char *data, int len)
{
	// Send only if network is active
	if(networkActive)
	{
        // Use this to store the number of bytes actually written to the TLS connection.
        int c;

		printf("WRITING TO CLIENT\n");
 
        if(tls_conn != NULL) {
			//SSLWrite function to wrtie data to the network.
			//handleNetworkData function sets tls_conn to point to the TLS connection we want to write to
			c = sslWrite(tls_conn, data, len);

        }

		// Network is still active if we can write more then 0 bytes.
		networkActive = (c > 0);
	}
}

/*
This functions handles the incoming commands of different types

@param conn The TLS connection we set up
@param buffer The buffer used to store the command & paramters
**/
void handleCommand(void *conn, const char *buffer)
{
	// The first byte contains the command
	char cmd = buffer[1];
	uint32_t cmdParam[2];

	// Copy over the parameters.
	memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;
	commandPacket.params[0] = cmdParam[0];
	commandPacket.params[1] = cmdParam[1];

	printf("COMMAND RECEIVED: %c %d %d\n", cmd, cmdParam[0], cmdParam[1]);
	
	switch(cmd)
	{
		case 'f':
		case 'F':
			commandPacket.command = COMMAND_FORWARD;
			uartSendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			commandPacket.command = COMMAND_REVERSE;
			uartSendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			commandPacket.command = COMMAND_TURN_LEFT;
			uartSendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			commandPacket.command = COMMAND_TURN_RIGHT;
			uartSendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			uartSendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			uartSendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			uartSendPacket(&commandPacket);
			break;
		default:
			printf("Bad command\n");

	}
}

/*
This function is called when we receive response back from the Arduino
We check if a command type packet is sent before handling the command

@param conn The TLS connection we will write to
@param buffer The buffer from SSLread. buffer[0] is of type TNetconstant.
**/
void handleNetworkData(void *conn, const char *buffer, int len)
{
    /* Note: A problem with our design is that we actually get data to be written
        to the SSL network from the serial port. I.e. we send a command to the Arduino,
        get back a status, then write to the TLS connection.  So we do a hack:
        we assume that whatever we get back from the Arduino is meant for the most
        recent client, so we just simply store conn, which contains the TLS
        connection, in a global variable called tls_conn */

        tls_conn = conn; // This is used by sendNetworkData

	if(buffer[0] == NET_COMMAND_PACKET)
		handleCommand(conn, buffer);
}

/*
This function runs continously while we are reading in data from the serial communication
It stores received data into the buffer by calling another function handleNetworkData

@param conn The TLS connection we set up
**/
void *worker(void *conn)
{
	int len;

	char buffer[BUF_LEN];
	
	while(networkActive)
	{
		//SSL read into buffer
		len = sslRead(conn, buffer, sizeof(buffer));

		// As long as we are getting data, network is active
		networkActive=(len > 0);

		if(len > 0){
			handleNetworkData(conn, buffer, len);
		}
		else if(len < 0){
				perror("ERROR READING NETWORK: ");
			}
		usleep(1000 * 1000);
	}

    // Reset tls_conn to NULL.
    tls_conn = NULL;
    EXIT_THREAD(conn);
}

/*
This function sends a hello packet as a "handshake" during initialization
**/
void sendHello()
{
	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	uartSendPacket(&helloPacket);
}

int main()
{
	printf("Starting Alex Server\n");
	setupSerial();
    networkActive = 1;

	//createServer function called to do client authentication and to send Alex's certification
	createServer(KEY_FNAME, CERT_FNAME, PORTNUM, &worker, CA_CERT_FNAME, CLIENT_NAME, 1);
	// Start the uartReceiveThread. The network thread is started by createServer

	printf("\nALEX REMOTE SUBSYSTEM\n\n");

	

	printf("DONE. Sending HELLO to Arduino\n");
	sendHello();
	printf("DONE.\n");


    // Loop while the server is active
    
    while(server_is_running());
}
