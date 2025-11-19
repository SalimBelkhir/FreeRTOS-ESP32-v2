#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <driver/i2c_master.h>
#include <esp_ssd1306.h>
#include "dht.h"

#define LED1_GPIO 2
#define LED2_GPIO 4 
#define	PIN_GPIO_DHT22 25

#define SENSOR1_BIT (1<<0)
#define SENSOR2_BIT (1<<1)

#define BUFFER_SIZE 128

//static const char *TAG = "example" ;
int sensorData ;


#ifdef CONFIG_FREERTOS_EXAMPLE_QUEUE
QueueHandle_t testQueue ;
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_SEMAPHORE
SemaphoreHandle_t dataReadySemaphore ;
#endif 

#ifdef CONFIG_FREERTOS_EXAMPLE_MUTEX
SemaphoreHandle_t xMutex;
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_EVENT_GROUP
EventGroupHandle_t sensorEventGroup = NULL;
#endif

/*-------------Prototype--------------*/
void init_gpio(int gpio_num);

#ifdef CONFIG_FREERTOS_EXAMPLE_TASK
void Task_toggle(int gpio_num,int time_ms);
void blink_task_1(void *pvParam);
void blink_task_2(void *pvParam);
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_QUEUE
void Send_Task(void *pvParam);
void Receive_Task(void *pvParam);
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_SEMAPHORE
void ADCTask(void *pvParam);
void UARTTask(void *pvParam);
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_MUTEX
void LPTask(void *pvParam);
void HPTask(void *pvParam);
#endif

#ifdef CONFIG_FREERTOS_EXAMPLE_EVENT_GROUP
void Sensor1Task(void *pvParam);
void Sensor2Task(void *pvParam);
void ProcessingTask(void *pvParam);
#endif

/*-------------Configuration-----------*/

void init_gpio(int gpio_num){
	esp_err_t res = gpio_reset_pin(gpio_num);
	assert(res==ESP_OK);
	res = gpio_set_direction(gpio_num,GPIO_MODE_OUTPUT);
	assert(res==ESP_OK);
}
#if defined(CONFIG_FREERTOS_EXAMPLE_QUEUE) || defined(CONFIG_FREERTOS_EXAMPLE_MUTEX)
/* I2C Master */
static const i2c_master_bus_config_t i2c_master_bus_config = {
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true};
static i2c_master_bus_handle_t i2c_master_bus;

/* SSD1306 */
static const i2c_ssd1306_config_t i2c_ssd1306_config = {
    .i2c_device_address = 0x3C,
    .i2c_scl_speed_hz = 400000,
    .width = 128,
    .height = 64,
    .wise = SSD1306_TOP_TO_BOTTOM};
static i2c_ssd1306_handle_t i2c_ssd1306;
#endif
/*-------------Tasks--------------*/
#ifdef CONFIG_FREERTOS_EXAMPLE_TASK
void Task_toggle(int gpio_num,int time_ms){
	for(;;){
		gpio_set_level(gpio_num,1);
		vTaskDelay(time_ms/portTICK_PERIOD_MS);
		gpio_set_level(gpio_num,0);
		vTaskDelay(time_ms/portTICK_PERIOD_MS);
	}
}

void blink_task_1(void *pvParam)
{
    Task_toggle(LED1_GPIO,200);
}

void blink_task_2(void *pvParam)
{
    Task_toggle(LED2_GPIO,1000);
}
#endif
/*------------Queue-----------------*/
#ifdef CONFIG_FREERTOS_EXAMPLE_QUEUE
void Send_Task(void *pvParam){
    int sensor_data = 0;
    int16_t temperature, humidity;

    for(;;){
        char temp_str[16] ;
        char hum_str[16];
        esp_err_t res = dht_read_data(DHT_TYPE_AM2301, 17, &humidity, &temperature);
        ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2c_ssd1306_buffer_fill_space(&i2c_ssd1306, 2, 124, 20, 28, false)
        );
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,10,"sending data",false));
        snprintf(temp_str, sizeof(temp_str), "T:%.1fC", temperature / 10.0f);
        snprintf(hum_str, sizeof(hum_str), "H:%.1f%%", humidity / 10.0f);
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,20,temp_str,false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,64,20,hum_str,false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_to_ram(&i2c_ssd1306));
        xQueueSend(testQueue,&temperature,portMAX_DELAY);
        xQueueSend(testQueue,&humidity,portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }       
}

void Receive_Task(void *pvParam)
{
    int received_data = 0;
    int16_t rx_temp = 0;
    int16_t rx_humidity = 0;
    char temp_str[16];
    char hum_str[16];
    for(;;){
        if(xQueueReceive(testQueue,&rx_temp,portMAX_DELAY) && xQueueReceive(testQueue,&rx_humidity,portMAX_DELAY)){
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2c_ssd1306_buffer_fill_space(&i2c_ssd1306, 2, 124, 40, 48, false)
            );
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,30,"Received data",false));
            snprintf(temp_str, sizeof(temp_str), "T:%.1fC", rx_temp / 10.0f);
            snprintf(hum_str, sizeof(hum_str), "H:%.1f%%", rx_humidity / 10.0f);
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,40,temp_str,false));
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,64,40,hum_str,false));
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,0,"Queue Com:",false));            
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_to_ram(&i2c_ssd1306));
        }
    }        
}
#endif
/*------------Semaphore-------------*/
#ifdef CONFIG_FREERTOS_EXAMPLE_SEMAPHORE
void ADCTask(void *pvParam)
{
    for(;;)
    {
        sensorData++;
        printf("ADC Task: New data = %d\n",sensorData);

        xSemaphoreGive(dataReadySemaphore);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void UARTTask(void *pvParam)
{
    for(;;){
        if(xSemaphoreTake(dataReadySemaphore,portMAX_DELAY))
        {
            printf("UART Task: sending data = %d\n",sensorData);
        }
    }
}
#endif
/*------------Mutex-------------*/
#ifdef CONFIG_FREERTOS_EXAMPLE_MUTEX
void LPTask(void *pvParam)
{
    int sensor_data = 0;
    int16_t temperature, humidity;
    for(;;){
        if(xSemaphoreTake(xMutex,portMAX_DELAY))
        {
            char temp_str[16] ;
            char hum_str[16];
            esp_err_t res = dht_read_data(DHT_TYPE_AM2301, 17, &humidity, &temperature);
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2c_ssd1306_buffer_fill_space(&i2c_ssd1306, 2, 124, 20, 28, false)
            );
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,10,"LP Task",false));
            snprintf(temp_str, sizeof(temp_str), "T:%.1fC", temperature / 10.0f);
            snprintf(hum_str, sizeof(hum_str), "H:%.1f%%", humidity / 10.0f);
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,20,temp_str,false));
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,64,20,hum_str,false));
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_text(&i2c_ssd1306,2,0,"Mutex Com:",false));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_to_ram(&i2c_ssd1306));
            printf("low priority task is acquiring a shared resource with the priority : %u\n",uxTaskPriorityGet(NULL));
            vTaskDelay(1000 / portTICK_PERIOD_MS); //mocking a long operation 
            printf("low priority task is releasing display : P=%u\n",uxTaskPriorityGet(NULL));
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

//medium priority task is blocked when the low priority release it's data so it cannot preempt the high priority task

void HPTask(void *pvParam)
{
    uint32_t alert_count = 0;
    for(;;){
        vTaskDelay(200 / portTICK_PERIOD_MS);//time to let low p tasks cookin 
        if(xSemaphoreTake(xMutex,portMAX_DELAY))
        {
            char alert_str[32];
            snprintf(alert_str,sizeof(alert_str),"ALERT #%lu",alert_count++);
            ESP_ERROR_CHECK_WITHOUT_ABORT(
            i2c_ssd1306_buffer_fill_space(&i2c_ssd1306, 2, 124, 40, 48, false)
            );
            ESP_ERROR_CHECK_WITHOUT_ABORT(
            i2c_ssd1306_buffer_text(&i2c_ssd1306, 0, 30, "*** ALERT ***", false)
            );
            ESP_ERROR_CHECK_WITHOUT_ABORT(
            i2c_ssd1306_buffer_text(&i2c_ssd1306, 0, 40, alert_str, false)
            );
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_ssd1306_buffer_to_ram(&i2c_ssd1306));
            printf("High priority task :Got shared resource\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            printf("High Priority Task : Release shared resource \n");
            xSemaphoreGive(xMutex);
        }
    }
}
#endif
/*------------EventsGroup-------------*/
#ifdef CONFIG_FREERTOS_EXAMPLE_EVENT_GROUP
void Sensor1Task(void *pvParam)
{
    for(;;)
    {
        printf("Snesor1 is reading data\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);//reading data
        printf("Sensor1 finished reading data\n");

        xEventGroupSetBits(sensorEventGroup,SENSOR1_BIT);
        vTaskDelay(2000 / portTICK_PERIOD_MS);//duration until next cycle
    }
}
void Sensor2Task(void *pvParam)
{
    for(;;)
    {
        printf("Snesor2 is reading data\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Sensor2 finished reading data\n");

        xEventGroupSetBits(sensorEventGroup,SENSOR1_BIT);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void ProcessingTask(void *pvParam)
{
    for(;;)
    {
        printf("Processing Task: Waiting for both sensors...\n");

        xEventGroupWaitBits(sensorEventGroup,
                            SENSOR1_BIT | SENSOR2_BIT,
                            pdTRUE,   // Clear bits after waiting
                            pdTRUE,   // Wait for both bits
                            portMAX_DELAY);

        printf("Processing Task: Both sensors ready! Processing data...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}
#endif

void app_main(void)
{
    printf("Starting FreeRTOS tasks...\n");
    //ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus));
    //ESP_ERROR_CHECK(i2c_ssd1306_init(i2c_master_bus, &i2c_ssd1306_config, &i2c_ssd1306));
    #ifdef CONFIG_FREERTOS_EXAMPLE_TASK
    printf("Running: Basic Tasks (LED Blinking)\n");
    init_gpio(LED1_GPIO);
    init_gpio(LED2_GPIO);
    xTaskCreate(blink_task_1,"Blink1",2048,NULL,1,NULL);
    xTaskCreate(blink_task_2,"Blink2",2048,NULL,1,NULL);
    #endif 

    #ifdef CONFIG_FREERTOS_EXAMPLE_QUEUE
    printf("Running: Queue Communication\n");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus));
    ESP_ERROR_CHECK(i2c_ssd1306_init(i2c_master_bus, &i2c_ssd1306_config, &i2c_ssd1306));
    testQueue = xQueueCreate(5,sizeof(int));
    if (testQueue == NULL)
    {
        printf("Queue creation failed!\n");
        return;
    }
    xTaskCreate(Send_Task,"Send task",2048,NULL,1,NULL);
    xTaskCreate(Receive_Task,"Receive Task",2048,NULL,1,NULL);
    #endif

    #ifdef CONFIG_FREERTOS_EXAMPLE_SEMAPHORE
    dataReadySemaphore = xSemaphoreCreateBinary();

    if(dataReadySemaphore == NULL){
        printf("Semaphore creation failed \n");
        return;
    }

    xTaskCreate(ADCTask,"ADC Task",2048,NULL,1,NULL);
    xTaskCreate(UARTTask,"Uart Task",2048,NULL,1,NULL);
    #endif

    #ifdef CONFIG_FREERTOS_EXAMPLE_MUTEX

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus));
    ESP_ERROR_CHECK(i2c_ssd1306_init(i2c_master_bus, &i2c_ssd1306_config, &i2c_ssd1306));
    xMutex = xSemaphoreCreateMutex();

    if(xMutex == NULL)
    {
        printf("Mutex creation failed \n");
        return ;
    }
    xTaskCreate(LPTask, "LowPriority", 2048, NULL, 1, NULL);
    xTaskCreate(HPTask, "HighPriority", 2048, NULL, 3, NULL);
    #endif

    #ifdef CONFIG_FREERTOS_EXAMPLE_EVENT_GROUP
    sensorEventGroup = xEventGroupCreate();
    if(sensorEventGroup == NULL)
    {
        printf("Event Group Creation failed\n");
        return ;
    }
    xTaskCreate(Sensor1Task, "Sensor1", 2048, NULL, 1, NULL);
    xTaskCreate(Sensor2Task, "Sensor2", 2048, NULL, 1, NULL);
    xTaskCreate(ProcessingTask, "Processing", 2048, NULL, 2, NULL);
    #endif
}
