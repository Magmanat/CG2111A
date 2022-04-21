
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// delay for sending commands
#define COMMAND_DELAY 1000 * 1000// in microseconds

// Tells us that the network is running.
static volatile int networkActive=0;

/**
 * This function will return error messages depending
 * on the type of error that occurs after it reads in data 
 * received. 
 * 
 * @param buffer The buffer containing data that will be read in.
 * 
 */
void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

/**
 * This function would print a series of data once the Pi receives
 * the 'get status' command. The data includes the distances
 * Alex has moved in each direction at that point in time
 * and the distances of obstacles on Alex's left and right 
 * sides respectively, which is picked up by ultrasonic sensors.
 * 
 * @param buffer The buffer containing data that will be read in. 
 */
void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	
	// Added these two stats, to be able to check if Alex was close to either the left or the right walls.
	// The distance would be printed in centimeters
	printf("Left Ultrasound Distance:\t%d\n", data[10]);
	printf("Right Ultrasound Distance:\t%d\n", data[11]);
	printf("\n---------------------------------------\n\n");
}

/**
 * This function would receive messages from Alex and print it.
 * 
 * @param buffer The buffer containing data that will be read in.
 */
void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

//This function is not used.
void handleCommand(const char *buffer)
{

}

/**
 * This function will decide which error message to send out
 * depending on the type of packet it receives and the error encountered. 
 * 
 * @param buffer The buffer containing data that will be read in. 
 * @param len Length of the buffer.
 */
void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}
/**
 * This function writes to Alex, issuing commands to it
 * only while the network is active.
 * 
 * @param conn TLS connection that is being written to. 
 * @param buffer The buffer containing data that will be read in.
 * @param len Length of buffer.
 */
void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
	
		c = sslWrite(conn,buffer,sizeof(buffer));

			
		networkActive = (c > 0);
	}
}

/**
 * This function reads data from the server while the network 
 * connection is active.
 * 
 * @param conn TLS connection that is being written to.
 * 
 */
void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		len = sslRead(conn, buffer, sizeof(buffer));	
		printf("read %d bytes from server.\n", len);	

		networkActive = (len > 0);

		if(networkActive){
			handleNetwork(buffer, len);
		}
	}

	printf("Exiting network listener thread\n");
    
   
  	stopClient();
  	EXIT_THREAD(conn); 
}

/**
 * Empties extra uneeded input to avoid program from hanging. 
 * 
 */

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

/**
 * This function prompts Alex's controller to give parameters
 * which instructs Alex on the distance, direction, speed.
 * 
 * @param params 
 */
void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &params[0], &params[1]);
	flushInput();
}

/**
 * This function allows Alex's controller to input commands after being prompted.
 * 
 * @param conn TLS connection that is being written to.
 * 
 */
void *writerThread(void *conn)
{
	int quit=0;

	while(!quit)
	{
		char ch;
		
		// Delay is added, as sometimes when commands are sent too fast, the commands get buffered and
		// Alex no longer behaves as intended, requiring us to restart the Alex-server and Alex-client
		usleep(COMMAND_DELAY); 
		
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats q=exit)\n");
		// added quick commands, W, A, X, D to be able to traverse the maze quickly without inputting distance/degrees + speed
		printf("For fast movement, command (w=forward, x=reverse, a=turn left, d=turn right)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			//controlled movement
			case 'f':
			case 'F':
			case 'b':
			case 'B':
			case 'l':
			case 'L':
			case 'r':
			case 'R':
						getParams(params);
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			//fast movement
			case 's':
			case 'S':
			case 'c':
			case 'C':
			case 'g':
			case 'G':
					params[0]=0;
					params[1]=0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(conn, buffer, sizeof(buffer));
					break;
			case 'q':
			case 'Q':
				quit=1;
				break;
				
			// w/W will move forward 5cm at 70 power
			case 'w':
			case 'W':
				params[0] = 5;
				params[1] = 70;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'f';
				sendData(conn, buffer, sizeof(buffer));
				break;
			// x/X will move backward 5cm at 70 power
			case 'x':
			case 'X':
				params[0] = 5;
				params[1] = 70;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'b';
				sendData(conn, buffer, sizeof(buffer));
				break;
			// a/A will move leftward 20 degrees at 65 power (calibrated such that will not affect hector mapping)
			case 'a':
			case 'A':
				params[0] = 20;
				params[1] = 65;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'l';
				sendData(conn, buffer, sizeof(buffer));
				break;
			// d/D will move rightward 20 degrees at 65 power (calibrated such that will not affect hector mapping)
			case 'd':
			case 'D':
				params[0] = 20;
				params[1] = 65;
				
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'r';
				sendData(conn, buffer, sizeof(buffer));
				break;
			default:
				printf("BAD COMMAND\n");
		}
	}

	printf("Exiting keyboard thread\n");

	stopClient();
	EXIT_THREAD(conn);
    
}



#define SERVER_NAME "192.168.182.124"
#define CA_CERT_FNAME "signing.pem"
#define PORT_NUM 5000
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "alex.app.com"

/**
 * This function connects Alex to the cloud.
 * 
 * @param serverName The name of the server Alex is connecting to.
 * @param portNum The port number assigned for connection.
 */
void connectToServer(const char *serverName, int portNum)
{
   
	createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
   
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

	networkActive = 1;
	connectToServer(av[1], atoi(av[2]));

	while(client_is_running());
	printf("\nMAIN exiting\n\n");
}
