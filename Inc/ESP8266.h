/*
 * lcd1602.h
 *
 *  Created on: Jan 21, 2020
 *      Author: Controllerstech
 */

#ifndef ESP8266_H_
#define ESP8266_H_


void Send_String(char *data);

void Send_Request(char *data);

int Wait_For(char *nstr);

void ESP_init();

#endif /* ESP8266_H_ */
