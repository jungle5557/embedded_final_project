/*
 * ESP8266_HAL.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */


#include "UartRingbuffer.h"
#include "ESP_DATA_HANDLER.h"
#include "stdio.h"
#include "string.h"
#include "main.h"

extern UART_HandleTypeDef huart7;
//extern UART_HandleTypeDef huart3;

#define wifi_uart &huart7
#define ESP_ENABLE 1


char buffer[20];


char *home = "<!DOCTYPE html>\n\
		<html>\n\
		<body>\n\
		<h1>404</h1>\n\
		</body></html>";

char *farm_top =  "<!DOCTYPE html><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"3\"><html><head><style>body{margin: 0; height: 100vh; display: flex; justify-content: center; \
		flex-direction: column; font-size: 80px;}.flex{display: flex;}.center{align-items: center; justify-content: center;} \
		.col{flex-direction: column;}.row{flex-direction: row;}.c1, .c2{padding: 10px; text-align: center;}.c2{position: relative; width: 400px;} \
		.c2::after{position: absolute; content: ""; bottom: 0; left: 0; width: 100%; height: 1px; background-color: gray;} \
		.btn{margin: 20px;}.btn a{background-color: #91B493; position: relative; display: inline-block; color: #4A593D; padding: 10px 20px; \
		text-decoration: none; border-radius: 6px;}.btn a.off{background-color: #BDC0BA; color: #91989F;}.btn:hover{cursor: pointer;} \
		@media (min-width:1200px){.btn a{padding: 10px 20px;}body{font-size: 35px;}}</style> </head> <body> \
		<div class=\"flex center col\" > <h2 class=\"title\">智慧農場</h2></div><div class=\"center flex row\"> \
		<div class=\"btn\">";

/*****************************************************************************************************************************************/

void ESP_Init ()
{
	char data[80];

	Ringbuf_init();

#if ESP_ENABLE == 1
	Uart_sendstring("Test...\r\n", wifi_uart);
	/********* AT **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT\r\n", wifi_uart);
	while(!(Wait_for("OK\r\n", wifi_uart)));


	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CWMODE=2\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));


	/********* AT+CIPMUX **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CIPMUX=1\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));

	/********* AT+CIPSERVER **********/
	Uart_flush(wifi_uart);
	Uart_sendstring("AT+CIPSERVER=1,80\r\n", wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));

#endif
	//char pData[]="AT OK\r\n";
	//HAL_UART_Transmit(&huart3, (uint8_t *)pData, 7, 0xfff);

}


int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for(">", wifi_uart)));
	Uart_sendstring (str, wifi_uart);
	while (!(Wait_for("SEND OK", wifi_uart)));
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	return 1;
}

void Server_Handle (char *str, int Link_ID)
{
	char datatosend[2048] = {0};
	if (!(strcmp (str, "/farm")))
	{
		char localbuf[200];
		sprintf(datatosend, farm_top);
		uint8_t fan_on = 0;
		uint8_t light_on = 0;
		uint8_t water_on = 0;
		uint8_t tc = 0;
		uint8_t rd = 0;
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		uint8_t soil = 0;

		if(HAL_HSEM_Take(9, 0) == HAL_OK) {
			fan_on = shared_ptr->fan_on;
			light_on = shared_ptr->light_on;
			water_on = shared_ptr->water_on;
			tc = shared_ptr->tc;
			rd = shared_ptr->rd;
			r = shared_ptr->r;
			g = shared_ptr->g;
			b = shared_ptr->b;
			soil = shared_ptr->soil;
			HAL_HSEM_Release(9,0);
		}

		if(fan_on)
		{
			strcat (datatosend, "<a href=\"/fanon\">風扇</a>");
		}
		else
		{
			strcat (datatosend, "<a href=\"/fanon\" class=\"off\">風扇</a>");
		}
		strcat (datatosend, "</div><div class=\"btn\">");
		if(light_on)
		{
			strcat (datatosend, "<a href\"/lighto\">燈光</a>");
		}
		else
		{
			strcat (datatosend, "<a href\"/lighto\" class=\"off\">燈光</a>");
		}
		strcat (datatosend, "</div><div class=\"btn\">");
		if(water_on)
		{
			strcat (datatosend, "<a href=\"/wateron\">澆水</a>");
		}
		else
		{
			strcat (datatosend, "<a href=\"/wateron\" class=\"off\">澆水</a>");
		}

		strcat (datatosend, "</div></div><div class=\"center flex col\"><div class=\"flex row center\"><div class=\"td c1\">溫度:</div>");
		sprintf (localbuf, "<div class=\"c2\">%d</div>", tc);
		strcat (datatosend, localbuf);
		strcat (datatosend, "</div><div class=\"flex row center\"><div class=\"c1\">濕度:</div>");
		sprintf (localbuf, "<div class=\"c2\">%d</div>", rd);
		strcat (datatosend, localbuf);
		strcat (datatosend, "</div><div class=\"flex row center\"><div class=\"c1\">光線:</div>");
		sprintf (localbuf, "<div class=\"c2\">R:%d G:%d B:%d</div>", r, g, b);
		strcat (datatosend, localbuf);
		strcat (datatosend, "</div><div class=\"flex row center\"><div class=\"c1\">土壤:</div>");
		sprintf (localbuf, "<div class=\"c2\">%d</div>", soil);
		strcat (datatosend, localbuf);
		strcat (datatosend, "</div></body></html>");

		Server_Send(datatosend, Link_ID);
	}
	else
	{
		sprintf (datatosend, home);
		Server_Send(datatosend, Link_ID);
	}

}

void Server_Start (void)
{
	char buftostoreheader[128] = {0};
	char Link_ID;
	while (!(Get_after("+IPD,", 1, &Link_ID, wifi_uart)));

	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftostoreheader,wifi_uart)));
	if (Look_for("/farm", buftostoreheader) == 1)
	{
		Server_Handle("/farm",Link_ID);
	}
	else if (Look_for("/favicon.ico", buftostoreheader) == 1);

	else
	{
		Server_Handle("/ ", Link_ID);
	}
}
