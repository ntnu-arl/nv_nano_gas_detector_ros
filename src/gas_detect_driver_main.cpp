/*
Nevada Nano Methane / Flammable Gas Sensor ROS Driver
Written By Frank Mascarich
HEAVILY based on uartTest.c from flam_examples written by Nevada Nano
Publishes 3 topics:
Custom message with all data
standard float message with concentration
standard int message with gas ID
*/

#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

#include <ros/ros.h>

#include <termios.h>

#include <boost/thread.hpp>

#include "gas_detect_driver_node/gas_detect.h"

#include <visualization_msgs/Marker.h>

#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

extern "C" {
    #include "uart.h"
}

#define __MAIN_C

/* Defines ----------------------------------------------------------------------------------------------*/
/*
 * Conversion macros for switching between Little and Big Endian.
*/
#define SWAP16(num)        (((num & 0xff00) >> 8) | (num << 8))
#define SWAP32(num)        (((num & 0xff000000) >> 24) | ((num & 0x00ff0000) >> 8) | ((num & 0x0000ff00) << 8) | (num << 24))

/* Command Status */
#define UART_SUCCESS           0x00
#define UART_CRC_ERROR         0x01
#define UART_BAD_PARAM         0x02
#define UART_EXE_FAILED        0x03
#define UART_NO_MEM            0x04
#define UART_UNKNOWN_CMD       0x05
#define FLAMMABLE true
/* commands */
#define CMD_ANSWER       0x01
#define CMD_ENGDATA      0x02
#ifdef FLAMMABLE
#define CMD_CONC         0x03
#define CMD_ID           0x04
#endif

#define CMD_TEMP         0x21
#define CMD_PRES         0x22
#define CMD_REL_HUM      0x23
#define CMD_ABS_HUM      0x24

#define CMD_STATUS       0x41
#define CMD_VERSION      0x42

#define CMD_MEAS         0x61
#define CMD_SHUTDOWN     0x62
#define CMD_BOOTLOADER   0x63
#define CMD_WRITE_FW     0x64

#define RQST_HDR_LENGTH     sizeof(uartRqstHeader_t)
#define REPLY_HDR_LENGTH    sizeof(uartReplyHeader_t)
#define NUM_OF_CMDS         (sizeof(uart_cmds) / sizeof(uart_cmd_t))
#define UART_MAX_DATA_SIZE  (1024*5)    /* maximum packet:  header + payload */
#define ENGDATA_CHUNKSIZE   512         /* size of each chunk of engineering data */
#define FINAL_PACKET        0x8000      /* bit to indicate last chunk of engineering data */

#define GAS_NAME_LENGTH     64

/* Structure definitions --------------------------------------------------------------------------------*/
typedef struct {
  uint16_t cmdID;
  uint16_t length;
  uint16_t reserved;
  uint16_t cksum;
} uartRqstHeader_t;

typedef struct {
  uint8_t cmdID;
  uint8_t status;
  uint16_t length;
  uint16_t cksum;
} uartReplyHeader_t;

typedef struct {
  uint8_t cmdID;
  uint16_t req_size;   /* Request size */
  uint16_t res_size;   /* Response size */
  uint32_t (*func)(uint8_t cmdID, uint8_t *data, uint16_t size);
} uart_cmd_t;

typedef struct {
  uint8_t sw_w;
  uint8_t sw_x;
  uint8_t sw_y;
  uint8_t sw_z;
  uint8_t hw_w;
  uint8_t hw_x;
  uint8_t proto_w;
  uint8_t proto_x;
} uart_version_t;

#ifdef FLAMMABLE
typedef struct {
  uint32_t cycleCount;
  float concentration;
  uint32_t flamID;
  float temp;
  float pressure;
  float relHumidity;
  float absHumidity;
} answer_t;
#else
#error Need to define expected answer type!
#endif

typedef struct {
  uint16_t pktnum;
  uint16_t length;
  uint8_t data[ENGDATA_CHUNKSIZE];
} uart_engdata_t;

typedef struct {
  float temp;
  float pressure;
  float humidity;
  float absHumidity;
  float humidAirDensity;
} enviro_reply_t;

/* Functions --------------------------------------------------------------------------------------------*/
extern int openSerialPort(char *device, int port);
extern void closeSerialPort(int);
extern int readSerial(char *readBuffer, uint32_t bytesToRead);
extern uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue);
static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size);
static void DumpRqstHdr(uartRqstHeader_t *);
static void DumpReplyHdr(uartReplyHeader_t *);
static void DumpHexa(uint8_t *p, uint32_t len);

void get_data_cb(const ros::TimerEvent&);
void call_command(uint8_t command_ind, uint32_t value=0);
/* Variables --------------------------------------------------------------------------------------------*/
int uartFP;
uint32_t verbose = 0, hexdump = 0;
char *filename = NULL;
uart_cmd_t uart_cmds[] = {
  {CMD_ANSWER, 0, sizeof(answer_t), ReadAnswer},
  {CMD_MEAS, 1, 0, WriteByte},
#ifdef FLAMMABLE
  {CMD_CONC, 0, 4, ReadFloat},
  {CMD_ID, 0, 4, ReadInteger},
#endif
  {CMD_ENGDATA, 0, sizeof(uart_engdata_t), ReadEngData},
  {CMD_TEMP, 0, 4, ReadFloat},
  {CMD_PRES, 0, 4, ReadFloat},
  {CMD_REL_HUM, 0, 4, ReadFloat},
  {CMD_ABS_HUM, 0, 4, ReadFloat},
  {CMD_STATUS, 0, 1, ReadByte},
  {CMD_VERSION, 0, 8, ReadVersion},
  {CMD_SHUTDOWN, 0, 0, WriteByte},
  {CMD_BOOTLOADER, 0, 0, WriteByte}
};


/* ROS Variables ----------------------------------------------------------------------------------------*/
ros::Publisher gas_detect_msg_pub;
ros::Publisher gas_conc_pub;
ros::Publisher gas_id_pub;

int last_seq_num = -1;

void mySigintHandler(int sig)
{
  closeSerialPort(uartFP);
  ros::shutdown();
}

int main(int argc, char** argv) {
	// start the node w/o sig int handler
	ros::init(argc, argv, "gas_detector_driver_node", ros::init_options::NoSigintHandler);
	// Create a node
	ros::NodeHandle nh("~");
  // setup an exit handler to close the serial port
	signal(SIGINT, mySigintHandler);
	// get the name of the serial port, and the port number from the launch file
	std::string serial_port_device_name = "/dev/ttyUSB";
	nh.getParam("serial_port_name", serial_port_device_name);
	int serial_port_device_num = 0;
	nh.getParam("serial_port_num", serial_port_device_num);
	// init the publishers
	gas_detect_msg_pub = nh.advertise<gas_detect_driver_node::gas_detect>("/gas_detect/msg", 10);
	gas_conc_pub = nh.advertise<std_msgs::Float32>("/gas_detect/concentration", 10);
	gas_id_pub = nh.advertise<std_msgs::UInt32>("/gas_detect/gas_id", 10);
	ros::Duration(1.5).sleep();
	// setup the serial port
	const char *port_name_ptr = serial_port_device_name.c_str();
  // if it fails to open
	if ((uartFP = openSerialPort((char*)port_name_ptr, serial_port_device_num)) == -1) 
	{
    // spit an error
		ROS_ERROR("Failed to open %s%d: %s (%d)\n", port_name_ptr, serial_port_device_num, strerror(errno), errno);
    // shutdown the node
		ros::shutdown();
	}
	// if the serial port was setup successfully, spin ros
	else
	{
    // call the command to start continuous measurements
		call_command(0x61, 0x2);
    // setup a timer to query the sensor 
		ros::Timer get_data_timer = nh.createTimer(ros::Duration(2.0), get_data_cb);
    // spin ros
		ros::spin();	
	}
	return 0;
}
void call_command(uint8_t cmdID, uint32_t value)
{
	int c, ii;
	int portNumber = -1;
	char *device = NULL;
	uint32_t oper = 0, sts;
	uint8_t reply[UART_MAX_DATA_SIZE];
  // loop through the available commands
	for(ii = 0; ii < NUM_OF_CMDS; ii++) 
	{
    // if the current doesn't match the command, continue
		if(uart_cmds[ii].cmdID != cmdID)
		{
			continue;
		}
    // otherwise call the appropriate function pointer of the command
		if(uart_cmds[ii].req_size) 
		{
			sts = uart_cmds[ii].func(cmdID, (uint8_t *) &value, uart_cmds[ii].req_size);
		} 
		else if(uart_cmds[ii].res_size) 
		{
			sts = uart_cmds[ii].func(cmdID, reply, uart_cmds[ii].res_size);
		} 
		else 
		{
			sts = uart_cmds[ii].func(cmdID, NULL, 0);
		}
    // check the status (0 = OK)
    if(sts != 0)
    {
      // figure out which message occurred
      switch(sts)
      {
        case 0x01:
          ROS_ERROR("GAS SENSOR ERROR - Transmitted data failed CRC check");
          break;
        case 0x02:
          ROS_ERROR("GAS SENSOR ERROR - Illegal or bad parameters specified");
          break;
        case 0x03:
          ROS_ERROR("GAS SENSOR ERROR - Execution of command failed");
          break;
        case 0x04:
          ROS_ERROR("GAS SENSOR ERROR - Insufficient memory for operation");
          break;
        case 0x05:
          ROS_ERROR("GAS SENSOR ERROR - Unknown Command ID specified");
          break;
      }
    }
    // exit the function
    return;
	}
}
// timer callback function to get new data from sensor
void get_data_cb(const ros::TimerEvent&)
{
  // call the read register function
	call_command(0x1);
}

/*
** All functions below are mostly taken from the Nevada Nano FLAM UART Example
*/
static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uartRqstHeader_t header;
  uartReplyHeader_t reply;
  uint16_t cksum, rxCksum, length;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;

  if(payloadLen != 0) {
    if(payload == NULL) {
      printf("No payload given but payload lengh is non-zero\n");
      return 1;
    }
    cksum = crc_generate(payload, payloadLen, cksum);
  }
  header.cksum = cksum;

  if(verbose) {
    DumpRqstHdr(&header);
    if(hexdump)
      DumpHexa((uint8_t *) &header, RQST_HDR_LENGTH);
  }

  if(write(uartFP, (uint8_t *) &header, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }

  if(payloadLen) {
    if(hexdump) {
      printf("  Payload");
      DumpHexa(payload, payloadLen);
    }

    if(write(uartFP, payload, payloadLen) != payloadLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }
  }

  return 0;
}

static uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t rxCksum, cksum;
  int rxLen;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];
  rxLen = readSerial((char*)buffer, sizeof(uartReplyHeader_t));
  if(rxLen <= 0) {
    printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
    return 1;
  }

  reply = (uartReplyHeader_t *) buffer;
  if(rxLen < REPLY_HDR_LENGTH) {
    printf("Incomplete header received: %d bytes\n", rxLen);
    DumpReplyHdr(reply);
    printf("\n\n");
    return 1;
  }

  if(reply->length != 0) {  /* Is there a payload for this reply? */
    rxLen = readSerial((char*)&buffer[REPLY_HDR_LENGTH], reply->length);
    if(rxLen < reply->length) {
      printf("Failed to get reply payload: %s (%d)\n", strerror(errno),  errno);
      return 1;
    }
  }

  rxCksum = reply->cksum;
  reply->cksum = 0;  /* zero out checksum field */
  cksum = crc_generate(buffer, REPLY_HDR_LENGTH + reply->length, 0xFFFF);
  if(rxCksum != cksum) {
    printf("Checksum failed: expected 0x%x, received 0x%x\n", cksum, rxCksum);
    DumpReplyHdr((uartReplyHeader_t *) buffer);
    return 1;
  }

  reply->cksum = rxCksum;   /* restore received checksum */

  if(reply->cmdID != cmdID) {
    printf("cmdID mismatch: expected 0x%x, received 0x%x\n", cmdID, reply->cmdID);
    return 1;
  }

  if(reply->status != UART_SUCCESS) {
    printf("Command returned error status: 0x%x\n", reply->status);
    return 1;
  }

  if(verbose) {
    DumpReplyHdr(reply);
    if(hexdump)
      DumpHexa((uint8_t *) reply, REPLY_HDR_LENGTH);
  }

  if (reply->length == 0)
    return 0;  /* No payload, we are done. */

  if(payloadLen < reply->length) {
    printf("Buffer too small for payload (%d < %d)\n", payloadLen, reply->length);
    return 1;
  }

  memset(payload, 0, payloadLen);
  memcpy(payload, &buffer[REPLY_HDR_LENGTH], reply->length);
  if(hexdump) {
    printf("  Payload:");
    DumpHexa(payload, reply->length);
  }
  return 0;
}


static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  float *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (float *) data;
  printf("Command[0x%02x]: %f\n", cmdID, *value);

  return 0;
}

static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (uint32_t *) data;
  printf("Command[0x%02x]: %u\n", cmdID, *value);

  return 0;
}

static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  version = (uart_version_t *) data;
  printf("SW Version: %u.%u.%u.%u, HW Version: %u.%u, Protocol: %u.%u\n",
         version->sw_w, version->sw_x, version->sw_y, version->sw_z,
         version->hw_w, version->hw_x, version->proto_w, version->proto_x);
  return 0;
}

static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("%s\n", data);
  return 0;
}

static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size) {
  answer_t *answer;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  answer = (answer_t *) data;
	#ifdef FLAMMABLE
  		//printf("Cycle: %u\nGas: %d\nConcentration: %f\nTEMP: %f\nPRESS: %f\nREL_HUM: %f\nABS_HUM: %f\n",
         	//SWAP32(answer->cycleCount), answer->flamID, answer->concentration, answer->temp, answer->pressure, answer->relHumidity, answer->absHumidity);
		uint32_t cycle_num = SWAP32(answer->cycleCount);
    if(last_seq_num == cycle_num)
		{
			return 0;
		}
		gas_detect_driver_node::gas_detect msg;
		msg.header.seq = cycle_num;
		msg.header.stamp = ros::Time::now();
		msg.abs_hum = answer->absHumidity;
		msg.concentration = answer->concentration;
		msg.cycle = cycle_num;
		msg.gas_id = answer->flamID;
		msg.pres = answer->pressure;
		msg.rel_hum = answer->relHumidity;
		msg.temp = answer->temp;
		gas_detect_msg_pub.publish(msg);

		std_msgs::Float32 conc_msg;
		conc_msg.data = answer->concentration;
		gas_conc_pub.publish(conc_msg);

		std_msgs::UInt32 gas_id_msg;
		gas_id_msg.data = answer->flamID;
		gas_id_pub.publish(gas_id_msg);
    
    // update the last sequence number
    last_seq_num = cycle_num;
	#endif
  return 0;
}

static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size) {

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("Command[0x%02x]: 0x%x\n", cmdID, *data);

  return 0;
}

static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size) {
  if(uartSend(cmdID, data, size) != 0)
    return 1;
  
  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint16_t pktnum = 0, txPktnum, rxPktnum;
  uart_engdata_t *engdata;
  uint8_t *p;
  uint16_t crc = 0, length;
  FILE *fp;

  if(filename == NULL) {
    printf("Missing filename parameters ('-f' option)\n");
    return 0;
  }

  fp = fopen(filename, "ab");
  if(fp == NULL) {
    printf("File open (%s) failed: %s\n", filename, strerror(errno));
    return 1;
  }

  do {
    txPktnum = SWAP16(pktnum);  /* outbound -> Big Endian */
    if(uartSend(cmdID, (uint8_t*) &txPktnum, sizeof(txPktnum)) != 0)
      return 1;

    if(uartRecv(cmdID, data, size) != 0) {
      printf("Failed to read\n");
      return 1;
    }

    engdata = (uart_engdata_t *) data;
    rxPktnum = SWAP16(engdata->pktnum);  /* inbound -> Little Endian */
    rxPktnum &= ~FINAL_PACKET;
    if(rxPktnum != pktnum) {
      printf("Unexpected packet # received (%d vs %d)\n", pktnum, rxPktnum);
      break;
    }

    length = SWAP16(engdata->length);
    fwrite(engdata->data, ENGDATA_CHUNKSIZE, 1, fp);
    fflush(stdout);
    pktnum++;
  } while ((SWAP16(engdata->pktnum) & FINAL_PACKET) == 0);  /* Continue while FINAL_PACKET bit is not set */

  fclose(fp);
  return 0;
}

static void DumpRqstHdr(uartRqstHeader_t *rqst) {
  printf("----\nREQUEST:\n");
  printf("  Hdr Size: %d\n", (int)sizeof(uartRqstHeader_t));
  printf("  CmdID: 0x%x\n", rqst->cmdID);
  printf("  Length: %d\n", rqst->length);
  printf("  Reserved: 0x%x\n", rqst->reserved);
  printf("  Checksum: 0x%x\n", rqst->cksum);
}

static void DumpReplyHdr(uartReplyHeader_t *reply) {
  printf("----\nREPLY:\n");
  printf("  CmdID: 0x%x\n", reply->cmdID);
  printf("  Status: 0x%x\n", reply->status);
  printf("  Length: %d\n", reply->length);
  printf("  Checksum: 0x%x\n", reply->cksum);
}

static void DumpHexa(uint8_t  *p, uint32_t len) {
  int ii;

  for(ii = 0; ii < len; ii++) {
    if((ii % 8) == 0)
      printf("\n    [%02d]: ", ii);

    printf("0x%02x ", *p++);
  }
  printf("\n");
}
