/*
 * RPLidarA1.h
 *
 *  Created on: Des 25, 2022
 *      Author: Maulana Reyhan Savero
 */
 #include "main.h"
 #include <math.h>

 
 #ifndef RPLIDARA1_H_
 #define RPLIDARA1_H_
 
 #define LIDAR_START_FLAG 0xa5
#define LIDAR_NEXT_FLAG 0x5a

// Send Mode
#define LIDAR_SEND_SINGLE_RESPONSE 0x0
#define LIDAR_SEND_MULTIPLE_RESPONSE 0x1

// Command
#define LIDAR_STOP 0x25
#define LIDAR_RESET 0x40
#define LIDAR_SCAN 0x20
#define LIDAR_EXPRESS_SCAN 0x82
#define LIDAR_FORCE_SCAN 0x21
#define LIDAR_GET_INFO 0x50
#define LIDAR_GET_HEALTH 0x52
#define LIDAR_GET_SAMPLERATE 0x59
#define LIDAR_GET_CONF 0x84

// Size of Data Received
#define LIDAR_SIZE_GET_SCAN 5 // bytes
#define LIDAR_SIZE_GET_FORCE_SCAN 5
#define LIDAR_SIZE_GET_INFO 20
#define LIDAR_SIZE_GET_HEALTH 3
#define LIDAR_SIZE_GET_SAMPLERATE 4

// Lidar Conf Command
#define LIDAR_CONF_SCAN_MODE_COUNT 0x70
#define LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE 0x71
#define LIDAR_CONF_SCAN_MODE_MAX_DISTANCE 0x74
#define LIDAR_CONF_SCAN_MODE_ANS_TYPE 0x75
#define LIDAR_CONF_SCAN_MODE_TYPICAL 0x7C
#define LIDAR_CONF_SCAN_MODE_NAME 0x7F

// Nodes Reading
#define LIDAR_MAX_WRONG_NODES 20
#define Time_Transmit 100
#define Time_Receive 100

// Special Command 
#define LIDAR_AVG_OFFSET 0

typedef enum
{
  LIDAR_OK       = 0x00U,
  LIDAR_TIMEOUT    = 0x01U,
  LIDAR_NO_GOOD_ANS     = 0x02U,
  LIDAR_NO_GOOD_NODE  = 0x03U
} lidar_StatusTypeDef;

typedef struct __lidar_rec_Handle
{	/*utilities */
	UART_HandleTypeDef* huart;
	uint8_t buff[100];
	/* data  */
	uint8_t descriptor[7];
	uint8_t start_scan_flag;
	uint8_t quality;
	float AVG[8];
	float angle;
	float distance;
	float degA[45];
	float degB[45];
	float degC[45];
	float degD[45];
	float degE[45];
	float degF[45];
	float degG[45];
	float degH[45];
} lidar_HandleTypeDef;

typedef struct{
	double distance[360];
	double angle[360];
}lidar_distance_angle_t;

typedef struct{
	uint8_t descriptor[7];
	uint8_t Model;
	uint8_t Firmware_Minor;
	uint8_t Firmware_Major;
	uint8_t Hardware;
	uint8_t Serial_Number[16];
} lidar_info_response_t;

typedef struct{
	uint8_t descriptor[7];
	uint8_t Status;
	uint16_t Error_Code;
} lidar_health_response_t;


typedef struct{
	uint8_t descriptor[7];
	uint16_t Time_Standart;
	uint16_t Time_Express;
} lidar_samplerate_response_t;

typedef struct{
	uint8_t scan_mode;
	uint8_t time_persample;
	uint8_t max_distance;
	uint8_t ans_type;
	uint8_t id_lidar;
	uint8_t scan_name;
} lidar_conf_t;

typedef struct{
	double dt1[75];
	double dt2[75];
	double dist1[75];
	double dist2[75];
}lidar_cabin_t;

typedef struct{
	double distance1[360];
	double angle1[360];
	double distance2[360];
	double angle2[360];
}lidar_real_data_t;

typedef struct{
	double angleToDistance[360];
}lidar_angletodata_t;

typedef struct{
	int axis_x[360];
	int axis_y[360];
}lidar_plot_t;

typedef struct{
	uint8_t response[4000];
	uint8_t Checksum;
	lidar_cabin_t Cabin;
	lidar_real_data_t Data;
	double Start_Angle[50];
	uint8_t counter;
	uint8_t flag;
	uint8_t flag_measuring;
	uint8_t total_cabin;
} lidar_data_scan_t;

// function
void lidar_setup(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_start_scan(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_read_scan(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_get_point(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_read_node(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_check_node(lidar_HandleTypeDef* lidar);
lidar_StatusTypeDef lidar_get_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health);
lidar_StatusTypeDef lidar_read_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health);
lidar_StatusTypeDef lidar_get_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info);
lidar_StatusTypeDef lidar_read_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info);
lidar_StatusTypeDef lidar_get_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate);
lidar_StatusTypeDef lidar_read_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate);
lidar_StatusTypeDef lidar_get_scanmode(lidar_HandleTypeDef* lidar, lidar_conf_t* conf);
lidar_StatusTypeDef lidar_get_express_scan(lidar_HandleTypeDef* lidar, lidar_data_scan_t* scan);
lidar_StatusTypeDef lidar_read_express_scan(lidar_HandleTypeDef* lidar, lidar_data_scan_t* scan);

 #endif
