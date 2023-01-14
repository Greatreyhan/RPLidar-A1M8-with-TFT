/*
 * RPLidarA1.c
 *
 *  Created on: Des 25, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "RPLidarA1.h"

void lidar_setup(lidar_HandleTypeDef* lidar){
	uint8_t rplidar_stop_msg[] = {LIDAR_START_FLAG,LIDAR_STOP};
	
	HAL_UART_Transmit(lidar->huart, rplidar_stop_msg, 2, Time_Transmit);
	HAL_Delay(20);
}

lidar_StatusTypeDef lidar_start_scan(lidar_HandleTypeDef* lidar){
	uint8_t rplidar_scan_msg[] = {0xA5, 0x20};
	
	HAL_UART_Transmit(lidar->huart, rplidar_scan_msg, 2, Time_Transmit);
	uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 7, Time_Receive);
	if(resp == HAL_OK){
		return lidar_read_scan(lidar);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_scan(lidar_HandleTypeDef* lidar){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x05 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x40 && lidar->buff[6]==0x81){
		for(uint8_t i =0; i < 7; i++){
			lidar->descriptor[i] = lidar->buff[i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_check_node(lidar_HandleTypeDef* lidar){
	if((lidar->buff[1]&0x01)&((lidar->buff[0]^(lidar->buff[0]>>1))&0x01)){
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_NODE;
}

lidar_StatusTypeDef lidar_read_node(lidar_HandleTypeDef* lidar){
	uint8_t status = HAL_UART_Receive(lidar->huart, lidar->buff, 5, Time_Receive);
	if(status == HAL_OK){
		return lidar_check_node(lidar);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_get_point(lidar_HandleTypeDef* lidar){
	lidar_StatusTypeDef status;
	uint8_t wrong_nodes = 0;
	do{
		status = lidar_read_node(lidar);
		wrong_nodes++;
	}
	while((status==LIDAR_NO_GOOD_NODE)&&(wrong_nodes<LIDAR_MAX_WRONG_NODES));
	
	if(status == LIDAR_OK){
		lidar->angle = ((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0; //deg
		lidar->distance = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10; // centimeters
		lidar->start_scan_flag = lidar->buff[0] & 0x1;
		lidar->quality = lidar->buff[0]>>2;
		
		// memperoleh seperempat data sudut 45
		if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (0+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (45+LIDAR_AVG_OFFSET) )){
			lidar->degA[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-LIDAR_AVG_OFFSET)] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 90
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (45+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (90+LIDAR_AVG_OFFSET) )){
			lidar->degB[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+45))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 135
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (90+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (135+LIDAR_AVG_OFFSET) )){
			lidar->degC[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+90))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 180
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (135+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (180+LIDAR_AVG_OFFSET) )){
			lidar->degD[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+135))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 225
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (180+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (225+LIDAR_AVG_OFFSET) )){
			lidar->degE[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+180))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 270
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (225+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (270+LIDAR_AVG_OFFSET) )){
			lidar->degF[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+225))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 315
		else if((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (270+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (315+LIDAR_AVG_OFFSET) )){
			lidar->degG[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+270))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
		}
		// memperoleh seperempat data sudut 360
		else if(((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (315+LIDAR_AVG_OFFSET) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (360+LIDAR_AVG_OFFSET) )) || ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) >= (0) && ((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0) <= (LIDAR_AVG_OFFSET) ))){
			if(((int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+315))) > 0){
				lidar->degH[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)-(LIDAR_AVG_OFFSET+315))] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
			}
			else{
				lidar->degH[(int)floor((((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0)+LIDAR_AVG_OFFSET)] = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/10;
			}
		}
	}
	
	return status;
}

lidar_StatusTypeDef lidar_get_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health){
		uint8_t rplidar_health_msg[] = {0xA5, 0x52};
		HAL_UART_Transmit(lidar->huart, rplidar_health_msg, 2, Time_Transmit);
		uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 10, Time_Receive);
		if(resp == HAL_OK){
			return lidar_read_health(lidar, health);
		}
		return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_health(lidar_HandleTypeDef* lidar, lidar_health_response_t* health){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x03 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x06){
		for(uint8_t i =0; i < 7; i++){
			health->descriptor[i] = lidar->buff[i];
		}
		health->Status =  lidar->buff[7];
		health->Error_Code = lidar->buff[8] | (lidar->buff[9] << 8);
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_get_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info){
	uint8_t rplidar_info_msg[] = {0xA5, 0x50};
	HAL_UART_Transmit(lidar->huart, rplidar_info_msg, 2, Time_Transmit);
	uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 27, Time_Receive);
	if(resp == HAL_OK){
		return lidar_read_info(lidar,info);
	}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_info(lidar_HandleTypeDef* lidar, lidar_info_response_t* info){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x14 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x04){
		for(uint8_t i =0; i < 7; i++){
			info->descriptor[i] = lidar->buff[i];
		}
		info->Model = lidar->buff[7];
		info->Firmware_Minor = lidar->buff[8];
		info->Firmware_Major = lidar->buff[9];
		info->Hardware = lidar->buff[10];
		for(uint8_t i=0;i<16;i++){
			info->Serial_Number[i] = lidar->buff[11+i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_get_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate){
		uint8_t rplidar_samplerate_msg[] = {0xA5, 0x59};
		HAL_UART_Transmit(lidar->huart, rplidar_samplerate_msg, 2, Time_Transmit);
		uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 11, Time_Receive);
		if(resp == HAL_OK){
			return lidar_read_samplerate(lidar, samplerate);
		}
		return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_read_samplerate(lidar_HandleTypeDef* lidar, lidar_samplerate_response_t* samplerate){
	if(lidar->buff[0]==0xa5 && lidar->buff[1]==0x5a && lidar->buff[2]==0x04 && lidar->buff[3]==0x00 && lidar->buff[4]==0x00 && lidar->buff[5]==0x00 && lidar->buff[6]==0x15){
		for(uint8_t i =0; i < 7; i++){
			samplerate->descriptor[i] = lidar->buff[i];
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}

lidar_StatusTypeDef lidar_get_scanmode(lidar_HandleTypeDef* lidar, lidar_conf_t* conf){
	uint8_t rplidar_conf_msg[] = {0xA5, 0x84};
	HAL_UART_Transmit(lidar->huart, rplidar_conf_msg, 3, Time_Transmit);
	uint8_t resp = HAL_UART_Receive(lidar->huart, lidar->buff, 100, 1000);
	if(resp == HAL_OK){
			return LIDAR_OK;
		}
	return LIDAR_TIMEOUT;
}

lidar_StatusTypeDef lidar_get_express_scan(lidar_HandleTypeDef* lidar, lidar_data_scan_t* scan){
	uint8_t rplidar_scan_msg[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00,0x00, 0x22};
	HAL_UART_Transmit(lidar->huart, rplidar_scan_msg, 9, 1000);
	uint8_t resp = HAL_UART_Receive(lidar->huart, scan->response, 4000, 2000);
	if(resp == HAL_OK){
			return lidar_read_express_scan(lidar, scan);
		}
	return LIDAR_TIMEOUT;
		
}

lidar_StatusTypeDef lidar_read_express_scan(lidar_HandleTypeDef* lidar, lidar_data_scan_t* scan){

	int counterCabin = 0;
	int counterData = 0;
	int diffData = 0;
	if(scan->response[0]==0xa5 && scan->response[1]==0x5a && scan->response[2]==0x54 && scan->response[3]==0x00 && scan->response[4]==0x00 && scan->response[5]==0x40 && scan->response[6]==0x82){
		
		// get test flag
			if(scan->counter > 2){
				scan->flag_measuring = 0x01;
			}
		
				
		// Scan for Sync Data
		for(int i = 0; i < sizeof(scan->response); i++){
			
			// filter for sync1 and sync2
			if((scan->response[i] >> 4 ) == 0xA && (scan->response[i+1] >> 4) == 0x5){
				
				if(diffData == 0 && counterCabin != 0){
					diffData = counterCabin;
				}
				
				counterCabin = 0;
					
				//get checksum 
				scan->Checksum = (scan->response[i] & 0xF) | ((scan->response[i+1] & 0xF) << 4);
				
				
				//get start angle
				scan->Start_Angle[scan->counter] = (double)(((scan->response[i+3] & 0x7F) << 8) | scan->response[i+2])/64.00;
				scan->counter = scan->counter+1;
				
				//get flag
				scan->flag = (scan->response[i+3] >> 7)& 0x01;
						
				//Get for cabin size 5 byte 15 number
				for(int j = 0; j < 15*5; j = j + 5 ){
					
					// check if contain sycn1 & 2
					if((scan->response[i+j+4] >> 4 ) == 0xA && (scan->response[i+j+5] >> 4) == 0x5){
						break;
					}
					
					// evaluated recent data dist & angle
					if(scan->Start_Angle[counterData+1] > 0.1){
						
						// get distance
						scan->Data.distance1[counterData+diffData] = ((scan->response[i+j+4] >> 2) & 0x3F) | (scan->response[i+j+5] << 8);
						scan->Data.distance2[counterData+diffData] = ((scan->response[i+j+6] >> 2) & 0x3F) | (scan->response[i+j+7] << 8);
						// get real angle
						scan->Data.angle1[counterData] = scan->Start_Angle[counterData] + (((scan->Start_Angle[counterData+1] - scan->Start_Angle[counterData])*counterData)/32) - scan->Cabin.dt1[counterData];;
						scan->Data.angle2[counterData] = scan->Start_Angle[counterData] + (((scan->Start_Angle[counterData+1] - scan->Start_Angle[counterData])*counterData)/32) - scan->Cabin.dt2[counterData];						
						counterData++;
					}
								
					
					scan->Data.distance1[counterData] = ((scan->response[i+j+4] >> 2) & 0x3F) | (scan->response[i+j+5] << 8);
					scan->Data.distance2[counterData] = ((scan->response[i+j+6] >> 2) & 0x3F) | (scan->response[i+j+7] << 8);
					scan->Cabin.dt1[counterCabin] = (scan->response[i+j+8] & 0xF) | ((scan->response[i+j+4] & 0x3) << 4);
					scan->Cabin.dt2[counterCabin] = ((scan->response[i+j+8] >> 4) & 0xF) | ((scan->response[i+j+6] & 0x3) << 4);
					
					counterCabin++;					
				}
				

			}
			
		}
		return LIDAR_OK;
	}
	return LIDAR_NO_GOOD_ANS;
}