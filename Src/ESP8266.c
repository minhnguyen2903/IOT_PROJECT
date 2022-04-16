#include "LCD1602.h"
#include "main.h"
#include "ESP8266.h"
#include <string.h>
#include <stdlib.h>


#define MAX_BUFFER_SIZE 50
extern UART_HandleTypeDef huart1;
extern char Rx_data[MAX_BUFFER_SIZE];
extern int waitForDataBack;
extern int timeout;
extern int requestId;
extern int request_ok;
extern int request_fail;
extern int request_reconnect;


void Send_String(char *data)
{
  //		memset(Rx_data, 0, MAX_BUFFER_SIZE);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, strlen(data));
  HAL_Delay(50);
  waitForDataBack = 1;
  timeout = 0;
}

void Send_Request(char *data)
{
  memset(Rx_data, 0, MAX_BUFFER_SIZE);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, strlen(data));
  HAL_Delay(50);
  waitForDataBack = 1;
  timeout = 0;
}

int Wait_For(char *nstr)
{
  HAL_Delay(1);
  timeout++;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  int len = strlen(nstr);
  int so_far = 0;
  int i = 0;
  while (i < MAX_BUFFER_SIZE)
  {
    int temp = i;
    if (Rx_data[i] == nstr[so_far])
    {
      while (so_far < len)
      {
        if (Rx_data[temp % MAX_BUFFER_SIZE] == nstr[so_far])
        {
          temp++;
          so_far++;
          if (so_far == len)
          {
            timeout = 0;
            return i;
          }
        }
        else
        {
          so_far = 0;
          break;
        }
      }
    }
    i++;
  }
  return 0;
}

void ESP_init()
{
espResetInit:
  lcd_clear();
  lcd_send_string("RESETING");

  Send_String("+++");
  HAL_Delay(200);
  Send_String("+++AT+RST\r\n");
  for (int i = 0; i < 5; i++)
  {
    HAL_Delay(2000);
    lcd_send_string(".");
  }

  Send_String("AT+CWMODE=1\r\n"); // thiet lap che do lam viec station
  while (!Wait_For("AT+CWMODE=1\r\r\n\r\nOK"))
  {
    if (timeout > 5000)
    {
      timeout = 0;
      goto espResetInit;
    }
  }
  lcd_clear();
  lcd_send_string("CONNECTING");
  lcd_put_cur(1, 0);
  lcd_send_string("TO WIFI");
  HAL_Delay(100);
  Send_String("AT+CIPMUX=0\r\n"); // thiet lap che do truyen don kenh
  while (!Wait_For("AT+CIPMUX=0\r\r\n\r\nOK"))
  {
    if (timeout > 5000)
    {
      timeout = 0;
      goto espResetInit;
    }
  };
  HAL_Delay(100);
  Send_String("AT+CWJAP=\"hehe\",\"00000000\"\r\n"); // ket noi wifi
  while (!Wait_For("WIFI GOT IP\r\n\r\nOK"))
  {
    if (timeout > 15000)
    {
      timeout = 0;
      goto espResetInit;
    }
  };
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("WIFI CONNECTED");
  HAL_Delay(1000);
  Send_String("AT+CIPSTART=\"TCP\",\"192.168.48.102\",8163\r\n"); // ket noi server
  while (!Wait_For("CONNECT\r\n\r\nOK"))
  {
    if (timeout > 5000)
    {
      timeout = 0;
      goto espResetInit;
    }
  };
  lcd_clear();
  lcd_send_string("SERVER CONNECTED");
  Send_String("AT+CIPMODE=1\r\n"); // thiet lap che do lam viec station
  while (!Wait_For("AT+CIPMODE=1\r\r\n\r\nOK"))
  {
    if (timeout > 5000)
    {
      timeout = 0;
      goto espResetInit;
    }
  }
  Send_String("AT+CIPSEND\r\n"); // thiet lap che do lam viec station
  HAL_Delay(200);
}
