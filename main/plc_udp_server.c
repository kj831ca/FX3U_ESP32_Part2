/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "PLCModBus/PLCModBus.h"

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";

static PLCUartSettings uartSetting = {UART_NUM_2,GPIO_NUM_17,GPIO_NUM_16};
QueueHandle_t xUartSendQueue; 
QueueHandle_t xUdpResponseQueue;
static ModBusFrame modBusFrame;
static PLCCommand plcCommand;

struct UDPMessage
{
    int id;
    char msg[64];
};

static void udp_plc_response_cb(void * arg)
{
    ModBusFrame *frame = (ModBusFrame *)arg;
    static struct UDPMessage plcMessage;
    struct UDPMessage *pxMyMessage;


    pxMyMessage = &plcMessage;
    uint32_t value, reg_number ;
    sprintf(plcMessage.msg,"Undefine Command..");
    switch(frame->command)
    {
        case SET_M:
            ESP_LOGI("CB","SM OK");
            sprintf(plcMessage.msg,"SM=%d",frame->data[2]);
            break;
        case SET_D:
            ESP_LOGI("CB","SD OK");
            sprintf(plcMessage.msg,"SD=OK");
            break;
        
        case READ_D:
            reg_number = (frame->data[0]<<8) + frame->data[1];
            if(frame->data_length == 4)
            {
                value = (frame->data[2] << 8) + frame->data[3];
                sprintf(plcMessage.msg,"D%d=%d",reg_number,(uint16_t)value);
            } else if(frame->data_length == 6)
            {
                value = (frame->data[2] << 8) + frame->data[3];
                ESP_LOGI("CB_W","Vaule low nibble %d", value );
                value += ((frame->data[4] <<8 ) + frame->data[5]) << 16 ;
                ESP_LOGI("CB_W","Vaule High nibble %d", value );
                sprintf(plcMessage.msg,"W%d=%d",reg_number,(int32_t)value);
            }
            ESP_LOGI("CB","%s",plcMessage.msg);
        break;
        case READ_M:
            reg_number = (frame->data[0]<<8) + frame->data[1];
            value = frame->data[2] ;
            sprintf(plcMessage.msg,"M%d=%d",reg_number,value);
            ESP_LOGI("CB","%s",plcMessage.msg);
        break;
    }
    xQueueSend(xUdpResponseQueue,&pxMyMessage,10);

}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    //char *respQueue = "" ;
    struct UDPMessage *respQueue;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if(ParsingPLCCommand(rx_buffer,&plcCommand))
                {
                    //HandlePLCCommand(&plcCommand,&modBusFram);
                    ESP_LOGI("UDP","PLC Command %d %d %d",plcCommand.command,plcCommand.station_number, plcCommand.reg_number);
                    HandlePLCCommand(&plcCommand,&modBusFrame);
                    vTaskDelay(60/portTICK_PERIOD_MS);
                }
                int err = 0;
                if(xQueueReceive(xUdpResponseQueue, &respQueue,(TickType_t)10)== pdPASS)
                {
                    ESP_LOGI("UDP_CB","Queue Receive!!!!");
                    len = strlen(respQueue->msg);
                    ESP_LOGI("UDP_CB","Len = %d %s",len,respQueue->msg);
                    if(len > 0)
                        err = sendto(sock, respQueue->msg, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                }
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    ESP_LOGE(TAG,"Task exit... bye....");
    vTaskDelete(NULL);
}

static PLCMReg plcY04 = {.stationNumber=1, .address = 0}; //Blink Y04
static PLCDReg plcAInput0  = {.stationNumber=1, .address = 0}; //PLC D0 Register
static PLCD32Reg plcD02 = {.stationNumber =1, .address =2}; //D2 
//Demonstrate reading and control the PLC from ESP32..
static void plc_reading_task(void *pvParameter)
{
    InitPLCMReg(&plcY04);
    InitPLCDReg(&plcAInput0);
    InitPLCD32Reg(&plcD02);
    TickType_t prevTick =  xTaskGetTickCount();
    uint16_t prevAI_value = 0;
    int32_t  prevD32Val = 0 ;
    while(1)
    {
        ReadPLCMReg(&plcY04);
        if(plcY04.state)
        {
            SetPLCMReg(&plcY04,0);
        } else 
        {
            SetPLCMReg(&plcY04,1);
        }
        if(ReadPLCDReg(&plcAInput0))
        {
            if(plcAInput0.value != prevAI_value)
            {
                ESP_LOGI(TAG,"Analog Input0= %d",plcAInput0.value);
                prevAI_value = plcAInput0.value;
            }
        }

        if(ReadPLCD32Reg(&plcD02))
        {
            if(plcD02.value != prevD32Val)
            {
                ESP_LOGI(TAG,"D2= %d",plcD02.value);
                prevD32Val = plcD02.value;
            }
        }
        vTaskDelayUntil(&prevTick,500/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xUartSendQueue = InitPLCTask(&uartSetting);
    modBusFrame.read_cb = &udp_plc_response_cb ; //Register the callback function when we received response from PLC.
    
    xUdpResponseQueue = xQueueCreate(5,sizeof(struct UDPMessage *));

    xTaskCreate(plc_reading_task, "PLC_Main_Task",2048,NULL,4,NULL);

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

}
