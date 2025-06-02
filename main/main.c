#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "esp_log.h"
#include <math.h>

// Motor 1
#define MOTOR1_STEP_MOTOR_GPIO_EN       14
#define MOTOR1_STEP_MOTOR_GPIO_DIR      12
#define MOTOR1_STEP_MOTOR_GPIO_STEP     13
#define MOTOR1_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define MOTOR1_STEP_MOTOR_DISABLE_LEVEL 1 // DRV8825 is enabled on low level
#define MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define MOTOR1_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

// Motor 2
#define MOTOR2_STEP_MOTOR_GPIO_EN       6
#define MOTOR2_STEP_MOTOR_GPIO_DIR      4
#define MOTOR2_STEP_MOTOR_GPIO_STEP     5
#define MOTOR2_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define MOTOR2_STEP_MOTOR_DISABLE_LEVEL 1 // DRV8825 is enabled on low level
#define MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define MOTOR2_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
#define MOTOR_MAX_FREQ 1000

#define TXD1_PIN (GPIO_NUM_7)
#define RXD1_PIN (GPIO_NUM_8)
#define TXD2_PIN (GPIO_NUM_15)
#define RXD2_PIN (GPIO_NUM_16)
#define LEDC_OUTPUT_IO1          (2)
#define LEDC_OUTPUT_IO2          (3)

#define EN_PIN1 (GPIO_NUM_36)
#define EN_PIN2 (GPIO_NUM_37)

#define STOP 0
#define RUN  1

#define FORWORD    2
#define BACKWORD    3

#define RIGHT 4
#define LEFT 5

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (200) // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_TEST_CH_NUM       (2)

// uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {1024,2048,1024,2048};
// uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {512,1024,2048,3072,4096,5120,6144,7168};
volatile int32_t DutyCycle = 1024;

typedef struct {
	float RollFactor;    
	float PitchFactor;   
	float YawFactor;     
	float ThrottleFactor;   
	float ForwardFactor;   
	float LateralFactor;
} variable;

//====================================================================================================================
//Motor #    	 Roll Factor    Pitch Factor    Yaw Factor     Throttle Factor    Forward Factor    Lateral Factor  

float fmotor1[6]={	0,			0,			    1,  			0,					1,					0};
float fmotor2[6]={	0,			0,				-1,				0,					1,					0};
//=====================================================================================================================
volatile double motorOut[2]= {0,0};
volatile double motorPulsOut[2]= {0,0};
volatile double motorPrvPulsOut[2]= {0,0};

float motorDir[2]= {0,0};
float motorPrevDir[2]= {0,0};
double setpoint[6]={0,0,0,0,0,0},output[6]={0,0,0,0,0,0},pwmoutput[6]={0,0,0,0,0,0} ;
int position[3]= {0,0,0};

#define RX_BUF_SIZE        512
#define LINE_BUF_SIZE      512
// static const int TX_BUF_SIZE = 8;
char buffer[17];
char responseData[1024];
volatile int32_t SPEED = 200;
volatile bool armed = false;

volatile int cmdFlag = 1, state = 0,previousState = 0;;

volatile bool flagMotor1RUN = 0,flagMotor2RUN = 0,errorFlag = 0,flagCamRUN = 0,serialFlag = 0, accelFlag = 1;

static const char *TAG = "example";
static const char *MOTOR_TAG = "MOTOR";
rmt_channel_handle_t motor1_chan = NULL;
rmt_channel_handle_t motor2_chan = NULL;
nvs_handle_t my_handle;

//====Encoder===========


#define ENCODER1_CLK_PIN GPIO_NUM_18
#define ENCODER1_DT_PIN  GPIO_NUM_19

#define ENCODER2_CLK_PIN GPIO_NUM_20
#define ENCODER2_DT_PIN  GPIO_NUM_21

#define PPR 400  // Pulses per revolution for your encoder (adjust as needed)
#define WHEEL_DIAMETER 0.044  // Wheel diameter in meters
#define WHEEL_BASE_LENGTH 0.250  // Wheel base length (distance between the two wheels) in meters


volatile int encoder1_count = 0;
volatile int encoder1_count_last = 0;  // Store the last count value of encoder 1

volatile int encoder2_count = 0;
volatile int encoder2_count_last = 0;  // Store the last count value of encoder 2

// Variables for velocity calculation
volatile float velocity1 = 0.0f;
volatile float velocity2 = 0.0f;
volatile float combined_velocity = 0.0f;  // Variable for combined velocity
volatile float rpm1 = 0.0f;  // RPM for encoder 1
volatile float rpm2 = 0.0f;  // RPM for encoder 2
volatile float linear_velocity_left = 0.0f;
volatile float linear_velocity_right = 0.0f;
volatile float robot_linear_velocity = 0.0f;
volatile float robot_angular_velocity = 0.0f;  // Angular velocity in radians per second
TickType_t last_time = 0;  // Last time when velocity was computed

// Helper function to configure encoder pins
void configure_encoder_pins(gpio_num_t clk_pin, gpio_num_t dt_pin) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << clk_pin),
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << dt_pin);
    gpio_config(&io_conf);
}

// ISR for Encoder 1
static void IRAM_ATTR gpio_isr_handler_encoder1(void* arg) {
    int clk = gpio_get_level(ENCODER1_CLK_PIN);
    int dt = gpio_get_level(ENCODER1_DT_PIN);
    if (clk == dt) {
        encoder1_count++;
    } else {
        encoder1_count--;
    }
}

// ISR for Encoder 2
static void IRAM_ATTR gpio_isr_handler_encoder2(void* arg) {
    int clk = gpio_get_level(ENCODER2_CLK_PIN);
    int dt = gpio_get_level(ENCODER2_DT_PIN);
    if (clk == dt) {
        encoder2_count++;
    } else {
        encoder2_count--;
    }
}
//===========================


int sendData(const char* logName, const char* data);
int sendData1(const char* logName, const char* data);

void motorInit(void);
void motorRunFwd(void);
void motorRunBwd(void);
void motor1RunFwd(void);
void motor2RunFwd(void);
void motorSTOP(void);
void saveSPEED(void);
void saveBRIGHT(void);
#define axes_size 8
double array_axes[axes_size]={};


// Example JSON string received
const char *received_json = "{\"op\": \"publish\", \"topic\": \"/joystick_input\", \"msg\": {\"header\": {\"stamp\": {\"sec\": 1729144145, \"nanosec\": 169086574}, \"frame_id\": \"\"}, \"axes\": [0.0, 0.0, 0.0, -0.49803921580314636], \"buttons\": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}";

void updateMotor(void)
{   
    int j;

    // Map axis inputs
    setpoint[2] = array_axes[2];  // Possibly: Z-axis or rotation
    setpoint[4] = array_axes[1];  // Possibly: Y-axis or forward/back

    // Reset outputs
    motorOut[0] = 0.0f;
    motorOut[1] = 0.0f;

    // Calculate motor output using weighted sum
    for (j = 0; j < 6; j++)
    {
        motorOut[0] += fmotor1[j] * setpoint[j];
        motorOut[1] += fmotor2[j] * setpoint[j];
    }

    ESP_LOGI("MotorOut", "Motor1 raw output: %f", motorOut[0]);
    ESP_LOGI("MotorOut", "Motor2 raw output: %f", motorOut[1]);
    motorPrvPulsOut[0]= motorPulsOut[0];
    motorPrvPulsOut[1]= motorPulsOut[1];
    
    // Process Motor 1
    if (motorOut[0] >= 0)
    {
        motorDir[0] = FORWORD;
    }
    else
    {
        motorDir[0] = BACKWORD;
    }

    motorPulsOut[0] = (int32_t)(fmin(fabs(motorOut[0]) * 1000.0f, 1000.0f));
   

    ESP_LOGI((motorDir[0] == FORWORD) ? "1FORWARD" : "1BACKWARD", "Pulse: %lf", motorPulsOut[0]);

    // Process Motor 2
    if (motorOut[1] >= 0)
    {
        motorDir[1] = FORWORD;
    }
    else
    {
        motorDir[1] = BACKWORD;
    }

    motorPulsOut[1] = (int32_t)(fmin(fabs(motorOut[1]) * 1000.0f, 1000.0f));

    ESP_LOGI((motorDir[1] == FORWORD) ? "2FORWARD" : "2BACKWARD", "Pulse: %lf", motorPulsOut[1]);
}


// Function to decode JSON string and extract joystick data
void decode_joystick_json(const char *json_string)
{
    // Parse the JSON string into a cJSON object
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        ESP_LOGE("Joystick", "Error parsing JSON");
        return;
    }

    // Extract the topic field
    cJSON *topic = cJSON_GetObjectItem(root, "topic");
    if (topic && cJSON_IsString(topic)) {
        ESP_LOGD("Joystick", "Topic: %s", topic->valuestring);
        
        // Check if the topic is "/joystick_input"
        if (strcmp(topic->valuestring, "/joystick_input") != 0) {
            ESP_LOGD("Joystick", "Received message on an unexpected topic: %s", topic->valuestring);
            cJSON_Delete(root);
            return;  // Exit function if topic does not match
        }
    } else {
        ESP_LOGE("Joystick", "No valid topic found");
        cJSON_Delete(root);
        return;
    }

    // Extract the "msg" object
    cJSON *msg = cJSON_GetObjectItem(root, "msg");
    if (msg == NULL) {
        ESP_LOGE("Joystick", "No 'msg' object found.");
        cJSON_Delete(root);
        return;
    }

    // Extract header (timestamp and frame_id)
    cJSON *header = cJSON_GetObjectItem(msg, "header");
    if (header) {
        cJSON *stamp = cJSON_GetObjectItem(header, "stamp");
        if (stamp) {
            cJSON *sec = cJSON_GetObjectItem(stamp, "sec");
            cJSON *nanosec = cJSON_GetObjectItem(stamp, "nanosec");
            if (sec && cJSON_IsNumber(sec) && nanosec && cJSON_IsNumber(nanosec)) {
                ESP_LOGD("Joystick", "Timestamp: %d sec, %d nanosec", sec->valueint, nanosec->valueint);
            }
        }
        cJSON *frame_id = cJSON_GetObjectItem(header, "frame_id");
        if (frame_id && cJSON_IsString(frame_id)) {
            ESP_LOGD("Joystick", "Frame ID: %s", frame_id->valuestring);
        }
    }

    // Extract axes array
    cJSON *axes = cJSON_GetObjectItem(msg, "axes");
    if (axes && cJSON_IsArray(axes)) {
        // int axes_size = cJSON_GetArraySize(axes);
        ESP_LOGD("Joystick", "Axes:"); 
        for (int i = 0; i < axes_size; i++) {
            cJSON *axis_value = cJSON_GetArrayItem(axes, i);
            if (axis_value && cJSON_IsNumber(axis_value)) {
                // ESP_LOGI("Joystick", "  Axis %d: %f", i, axis_value->valuedouble);
                array_axes[i]=axis_value->valuedouble;
                // ESP_LOGI("Joystick", "  Axis %d: %f", i, array_axes[i]);
            }
        }
    }

    // Extract buttons array
    cJSON *buttons = cJSON_GetObjectItem(msg, "buttons");
    if (buttons && cJSON_IsArray(buttons)) {
        int buttons_size = cJSON_GetArraySize(buttons);
        ESP_LOGD("Joystick", "Buttons:");
        for (int i = 0; i < buttons_size; i++) {
            cJSON *button_value = cJSON_GetArrayItem(buttons, i);
            if (button_value && cJSON_IsNumber(button_value)) {
                ESP_LOGD("Joystick", "  Button %d: %d", i, button_value->valueint);
            }
        }
    }
    serialFlag=1;
    updateMotor();
    // Clean up and free the memory used by the cJSON object
    cJSON_Delete(root);
}

void initUART(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}
void initUART1(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void initUART2(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData1(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
// int sendData1(const char* logName, unsigned const char* data,const int len)
// {
//     // const int len = strlen(data);
//     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }
int sendData2(const char* logName, unsigned const char* data,const int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static int uart_read_line(uart_port_t uart_num, uint8_t *data, size_t max_len, int timeout_ms) {
    size_t len = 0;
    uint8_t byte;
    
    while (len < max_len - 1) {  // Reserve space for null terminator
        int read_len = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(timeout_ms));
        
        if (read_len > 0) {
            if (byte == '\n') {  // End of line detected
                break;
            }
            data[len++] = byte;
        } else {
            // Timeout or no data
            break;
        }
    }
    data[len] = '\0';  // Null-terminate the string
    return len;
}

static void rx_task(void *arg) {
    static const char *RX_TASK_TAG = "CONFIGURE";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t *line_data = (uint8_t *) malloc(LINE_BUF_SIZE + 1);
    if (line_data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");
        return;
    }

    while (1) {
        int line_len = uart_read_line(UART_NUM_0, line_data, LINE_BUF_SIZE, 500);

        if (line_len > 0) {
            ESP_LOGI(RX_TASK_TAG, "Read line: '%s'", line_data);
            char dataArray [1024];
            memcpy(dataArray,line_data, line_len);

            // Process the line data as needed
            // Example: Decode JSON or process the command
            decode_joystick_json((char *)dataArray);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
    free(line_data);
}

static void rx_task1(void *arg) {
    static const char *RX_TASK_TAG = "CONFIGURE";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t *line_data = (uint8_t *) malloc(LINE_BUF_SIZE + 1);
    if (line_data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");
        return;
    }

    while (1) {
        int line_len = uart_read_line(UART_NUM_1, line_data, LINE_BUF_SIZE, 500);

        if (line_len > 0) {
            ESP_LOGI(RX_TASK_TAG, "Read line: '%s'", line_data);
            char dataArray [1024];
            memcpy(dataArray,line_data, line_len);

            // Process the line data as needed
            // Example: Decode JSON or process the command
            decode_joystick_json((char *)dataArray);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
    free(line_data); 
}


void pwmLEDC(void *arg)
{
    int ch;
    int32_t CurrentDuty = 0;

    // Set the LEDC peripheral configuration
        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO1 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        },        
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO2 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        }        
    };
    ESP_LOGI(TAG, "Init LED ");
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    ESP_LOGI(TAG, "Setting LED duty cycle");
    //   for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
    //         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[ch].channel, DutyCycle));
    //         // Update duty to apply the new value
    //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[ch].channel));

    //   } 
    CurrentDuty=DutyCycle;
    while (true){
        if (DutyCycle!=CurrentDuty)
        {
            for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[ch].channel, DutyCycle));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[ch].channel));
            }
            CurrentDuty= DutyCycle;
        }
            vTaskDelay(300/ portTICK_PERIOD_MS);
            // ESP_LOGI(TAG, "LEDC task");
    }
}

void saveSPEED()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

    err = nvs_set_i32(my_handle, "SPEED",SPEED);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
void saveBRIGHT()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

    err = nvs_set_i32(my_handle, "BRIGHT",DutyCycle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
void reset_rmt_channel(rmt_channel_handle_t chan)
{
    ESP_ERROR_CHECK(rmt_disable(chan));
    vTaskDelay(2 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(rmt_enable(chan));
}

static void  stepper_motor_run(void *pvParameter)
{

// ===========Init stepper motor ==========================================

    ESP_LOGI(TAG, "Initialize EN + DIR GPIO for Motor2");
    gpio_config_t motor_en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = ((1ULL << MOTOR1_STEP_MOTOR_GPIO_DIR) | (1ULL << MOTOR1_STEP_MOTOR_GPIO_EN)|(1ULL << MOTOR2_STEP_MOTOR_GPIO_DIR)|(1ULL << MOTOR2_STEP_MOTOR_GPIO_EN)),
    };
    ESP_ERROR_CHECK(gpio_config(&motor_en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel for Motor 1");
    rmt_tx_channel_config_t motor1_tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR1_STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&motor1_tx_chan_config, &motor1_chan));

    ESP_LOGI(TAG, "Create RMT TX channel for Motor 2");
    rmt_tx_channel_config_t motor2_tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR2_STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&motor2_tx_chan_config, &motor2_chan));

    stepper_motor_uniform_encoder_config_t motor1_uniform_encoder_config = {
        .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
    };

    rmt_encoder_handle_t motor1_uniform_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor1_uniform_encoder_config, &motor1_uniform_encoder));

    stepper_motor_curve_encoder_config_t motor1_decel_encoder_config = {
        .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 10,
        .start_freq_hz = 100,
        .end_freq_hz = 0,
    };
 
    stepper_motor_uniform_encoder_config_t motor2_uniform_encoder_config = {
        .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t motor2_uniform_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor2_uniform_encoder_config, &motor2_uniform_encoder));

    ESP_LOGI(TAG, "Enable RMT channels");
    ESP_ERROR_CHECK(rmt_enable(motor1_chan));
    ESP_ERROR_CHECK(rmt_enable(motor2_chan));

    ESP_LOGI(TAG, "Spin motors");
    rmt_transmit_config_t tx_config = {
        .loop_count = -1,
    };
    
    int gain = 1;
    uint32_t uniform_speed1_hz = 0;
    uint32_t uniform_speed2_hz = 0;
    uint32_t offsetUniform_speed = 50;

    while(true) 
    {   
  
        if (FORWORD == motorDir[0])
        {
            ESP_LOGI(MOTOR_TAG, "Motor1 Moving Forword%lf", motorPulsOut[0]);
            // ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
            gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
        }
        if (BACKWORD == motorDir[0])
        {
            ESP_LOGI(MOTOR_TAG, "Motor1 Moving Backword%lf", motorPulsOut[0]);
            // ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
            gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
        }

        // Direction Motor 2
        if (FORWORD == motorDir[1])
        {
            ESP_LOGI(MOTOR_TAG, "Motor2 Moving Forword%lf", motorPulsOut[1]);
            // ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
            gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
        }
        if (BACKWORD == motorDir[1])
        {
            ESP_LOGI(MOTOR_TAG, "Motor2 Moving Backword%lf", motorPulsOut[1]);
            // ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
            gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE );
        }   
       
       
        // Optimized Motor 1 RMT Control for looped transmission
        if (motorPulsOut[0] >= offsetUniform_speed)
        {
            if (motorPrvPulsOut[0] != motorPulsOut[0])
            {
                ESP_LOGI(TAG, "Updating Motor1 speed: %d Hz", (int)motorPulsOut[0]);
                uniform_speed1_hz = (uint32_t)(gain * fmin(motorPulsOut[0], MOTOR_MAX_FREQ));

                reset_rmt_channel(motor1_chan);

                ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed1_hz, sizeof(uniform_speed1_hz), &tx_config));
            }
            flagMotor1RUN = 1;
        }
        else if (flagMotor1RUN)
        {
            ESP_LOGI(TAG, "Stopping Motor1 (Below threshold)");
            reset_rmt_channel(motor1_chan);
            motorPrvPulsOut[0] = 0;
            flagMotor1RUN = 0;
        }

        // Optimized Motor 2 RMT Control for looped transmission
        if (motorPulsOut[1] >= offsetUniform_speed)
        {
            if (motorPrvPulsOut[1] != motorPulsOut[1])
            {
                ESP_LOGI(TAG, "Updating Motor2 speed: %d Hz", (int)motorPulsOut[1]);
                uniform_speed2_hz = (uint32_t)(gain * fmin(motorPulsOut[1], MOTOR_MAX_FREQ));

                reset_rmt_channel(motor2_chan);

                ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed2_hz, sizeof(uniform_speed2_hz), &tx_config));
            }
            flagMotor2RUN = 1;
        }
        else if (flagMotor2RUN)
        {
            ESP_LOGI(TAG, "Stopping Motor2 (Below threshold)");
            reset_rmt_channel(motor2_chan);
            motorPrvPulsOut[1] = 0;
            flagMotor2RUN = 0;
        }

        motorPrvPulsOut[0] = motorPulsOut[0];
        motorPrvPulsOut[1] = motorPulsOut[1];
        motorPrevDir[0] = motorDir[0];
        motorPrevDir[1] = motorDir[1];

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}

void initPeripheral(void)
{
    initUART();
    initUART1();
    initUART2();
}

void initStorage(void)
{
 // Initialize NVS==============================================
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Done\n");
        err = nvs_get_i32(my_handle, "SPEED", &SPEED);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(TAG,"Done\n");
                ESP_LOGI(TAG,"SPEED = %ld\n", SPEED);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(TAG,"The value SPEED is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
        }
        // err = nvs_get_i32(my_handle, "Brightness", &DutyCycle);
        // switch (err) {
        //     case ESP_OK:
        //         ESP_LOGI(TAG,"Done\n");
        //         ESP_LOGI(TAG,"Dutycycle = %ld\n", DutyCycle);
        //         break;
        //     case ESP_ERR_NVS_NOT_FOUND:
        //         ESP_LOGE(TAG,"The value is not initialized yet!\n");
        //         break;
        //     default :
        //         ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
        // }
        //  vTaskDelay(100 / portTICK_PERIOD_MS);

        vTaskDelay(100 / portTICK_PERIOD_MS);
            // Close
        nvs_close(my_handle);
    }
}

void initencoder(void)
{
        // Configure Encoder 1 and Encoder 2
    configure_encoder_pins(ENCODER1_CLK_PIN, ENCODER1_DT_PIN);
    configure_encoder_pins(ENCODER2_CLK_PIN, ENCODER2_DT_PIN);

}
void velocity_task(void *pvParameters) {

    // Install ISR service and assign ISRs to the respective encoder pins
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER1_CLK_PIN, gpio_isr_handler_encoder1, NULL);
    gpio_isr_handler_add(ENCODER2_CLK_PIN, gpio_isr_handler_encoder2, NULL);
    char json_buffer[128]; // Buffer to hold the JSON string'

    while (1) {
        // Calculate velocity for Encoder 1 (counts per 100 ms)
        int count_diff1 = encoder1_count - encoder1_count_last;
        encoder1_count_last = encoder1_count;

        velocity1 = (float)count_diff1 * 10.0f;  // Scaling to counts per second (multiply by 10 for 100ms)
        // rpm1 =  (velocity1/PPR)*60;
        // ESP_LOGI(TAG, "rpm1: %.3f", rpm1);

        
        // Calculate velocity for Encoder 2 (counts per 100 ms)
        int count_diff2 = encoder2_count - encoder2_count_last;
        encoder2_count_last = encoder2_count;

        velocity2 = (float)count_diff2 * 10.0f;
        // rpm2 =  (velocity2/PPR)*60;
        // ESP_LOGI(TAG, "rpm2: %.3f", rpm2);


        // Calculate linear velocity of each wheel
        linear_velocity_left = (2.0f * M_PI * (WHEEL_DIAMETER / 2.0f) * velocity1) / PPR;
        linear_velocity_right = (2.0f * M_PI * (WHEEL_DIAMETER / 2.0f) * velocity2) / PPR;

        // Calculate the robot's linear velocity (V) - average of the left and right wheel velocities
        robot_linear_velocity = (linear_velocity_left - linear_velocity_right) / 2.0f;

        // Calculate the robot's angular velocity (ω)
        robot_angular_velocity = (linear_velocity_left + linear_velocity_right ) / WHEEL_BASE_LENGTH;
    
        // Log velocities of both encoders, the robot's linear and angular velocities
        ESP_LOGI(TAG, "Encoder 1 - Count: %d, Velocity: %.2f counts/sec, Linear Velocity: %.2f m/s", encoder1_count, velocity1, linear_velocity_left);
        ESP_LOGI(TAG, "Encoder 2 - Count: %d, Velocity: %.2f counts/sec, Linear Velocity: %.2f m/s", encoder2_count, velocity2, linear_velocity_right);
        ESP_LOGI(TAG, "Linear Velocity:   %.2f   m/s, Angular Velocity:   %.2f   rad/s", robot_linear_velocity, robot_angular_velocity);
        ESP_LOGI(TAG, "Robot Angular Velocity: %.2f rad/s", robot_angular_velocity);
        //  Safely format the JSON string
        snprintf(json_buffer, sizeof(json_buffer), 
         "{\"sensor\": \"Encoder\", \"data\": {\"linear_velocity\": %.2f, \"angular_velocity\": %.2f}}",
        robot_linear_velocity, robot_angular_velocity);
        sendData1("Encoder",json_buffer );
        ESP_LOGI(TAG, "Robot Velocity: %s", json_buffer); 
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100 ms
    }
}

void app_main(void)
{  
    esp_err_t res;
    initPeripheral();
    initencoder();
    initStorage();

    xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(rx_task1, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    
    xTaskCreate(pwmLEDC, "pwmLEDC_task", 1024 * 4, NULL, configMAX_PRIORITIES - 4, NULL);

    xTaskCreate(velocity_task, "velocity_task", 2*2048, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(stepper_motor_run, "stepper_motor_run",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    
    ESP_LOGI(TAG, "Initialization DONE");

    while (1) {
        // ESP_LOGD(TAG, "Looping");
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}