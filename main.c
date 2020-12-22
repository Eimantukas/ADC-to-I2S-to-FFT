#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_adc_cal.h"
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include "soc/uart_struct.h"
#include <math.h>
#include "esp_dsp.h"

//ADC definitions
#define DEFAULT_VREF        1100

//LEDc definitions
#define LEDC_HS_TIMER       LEDC_TIMER_0
#define LEDC_HS_MODE        LEDC_HIGH_SPEED_MODE
#define LEDC_CLK            LEDC_AUTO_CLK
#define LEDC_RESOLUTION     LEDC_TIMER_12_BIT
#define LEDC_RED_GPIO       (26)
#define LEDC_RED_CHANNEL    LEDC_CHANNEL_0
#define LEDC_GREE_GPIO      (25)
#define LEDC_GREE_CHANNEL   LEDC_CHANNEL_1
#define LEDC_BLU_GPIO       (27)
#define LEDC_BLU_CHANNEL    LEDC_CHANNEL_2
#define LEDC_TEST_CH_NUM    (3)
#define LEDC_TEST_DUTY      (4000)

//i2s definitions
#define I2S_NUM             I2S_NUM_0                                      //i2s number
#define I2S_FORMAT          I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB  //i2s sample rate
#define I2S_MODE            I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN
#define I2S_CHANNEL         I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_SAMPLES         1024
#define I2S_SAMPLE_BITS     I2S_BITS_PER_SAMPLE_16BIT
#define I2S_ADC_UNIT        ADC_UNIT_1                                     //I2S built-in ADC unit
#define I2S_ADC_CHANNEL     ADC1_CHANNEL_3                                 //I2S built-in ADC channel

//#define SET_PERI_REG_MASK
#define DUMP_INTERVAL 1000
#define SAMPLES 1024

//static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channelR = ADC_CHANNEL_3;
//static const adc_channel_t channelG = ADC_CHANNEL_6;
//static const adc_channel_t channelB = ADC_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
//static const adc_unit_t unit = ADC_UNIT_1;




const int NUM_SAMPLES = SAMPLES;
int samplingFrequency =  44000;

static const char *TAG = "main";
float x1[SAMPLES]; 
float wind[SAMPLES];
float y_cf[SAMPLES*2];
float* y1_cf = &y_cf[0];

static float real[SAMPLES];
static float imag[SAMPLES];




/*static void checkEfuse(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP)==ESP_OK)   
    {
        printf("eFuse Two Point supported\n");
    }
    else    
    {
        printf("eFuse Two Point not supported\n");
    }

    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF)==ESP_OK)    
    {
        printf("eFuse Vref supported\n");
    }
    else    
    {
        printf("eFuse Vref not supported\n");
    }
    
}*/

/*static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)    
    {
        printf("Characterized using Two Point Value\n");
    } 
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)    
    {
        printf("Charecterized using eFuse Vref\n");
    }
    else    
    {
        printf("Charecterized using default Vref\n");
    }    
}*/

void i2sConfig()
{
    const i2s_config_t i2s_config = 
        {
			.mode = (i2s_mode_t)(I2S_MODE),
			.sample_rate = samplingFrequency,  
			.dma_buf_len = NUM_SAMPLES,
            .dma_buf_count = 8,               
			.bits_per_sample = I2S_SAMPLE_BITS,
			.channel_format = I2S_CHANNEL,
			.use_apll = false,
			.communication_format = (i2s_comm_format_t)(I2S_FORMAT),
			.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,			
        };
        
    //adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_3);
    adc1_config_width(width);
    adc1_config_channel_atten(channelR, atten);
    i2s_adc_enable(I2S_NUM);    
    i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);    
    //SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);    
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);    
}



static void integerToFloat(int16_t *integer, float *vReal, float *vImag, uint16_t samples)

{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = (integer[i] >> 8);
        vImag[i] = 0.0;
    }
}

void fillBuffer ()
{
    int16_t i2sReadBuffer[NUM_SAMPLES];  
    size_t bytesRead;  
    
    i2s_read(I2S_NUM, (char*)i2sReadBuffer, NUM_SAMPLES, &bytesRead, portMAX_DELAY);
        
    //int samplesRead = bytesRead / 8;
     
    integerToFloat(i2sReadBuffer, real, imag, NUM_SAMPLES);
}

void app_main(void)
{ 
    esp_err_t ret;
    ESP_LOGI (TAG, "Start example.");
    ret = dsps_fft2r_init_fc32(real, NUM_SAMPLES);
    if (ret != ESP_OK)
    {
        ESP_LOGV(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }
    
    dsps_wind_hann_f32(wind, NUM_SAMPLES); //generate hann window
    
    
    for(int i=0; i < NUM_SAMPLES; i++)
    {
        y_cf[i*2+0]= real[i]*wind[i];
    }
    //FFT
    unsigned int start_b = xthal_get_ccount();
    dsps_fft2r_fc32(y_cf, NUM_SAMPLES);
    unsigned int end_b = xthal_get_ccount();

    dsps_bit_rev_fc32(y_cf, NUM_SAMPLES); //bit reverse
    dsps_cplx2reC_fc32(y_cf, NUM_SAMPLES);

    for(int i = 0; i < NUM_SAMPLES/2; i++)
    {
        y1_cf[i] = 10 * log10f((y1_cf[i*2+0] * y1_cf[i*2+0] + y1_cf[i*2+1] * y1_cf[i*2+1])/NUM_SAMPLES);
    }
    
    while(1)
    {
    ESP_LOGW(TAG, "Signal x1");    
    ESP_LOGI(TAG, "FFT for %i complex points take %i cycles", NUM_SAMPLES, end_b - start_b);
    ESP_LOGI(TAG, "End Example.");
    dsps_view(y1_cf, NUM_SAMPLES/2, 128, 20, -100, 40, '|');


        
    
   /* while (1)
    {
        uint16_t adc_readingR = 0;
        //uint32_t adc_readingG = 0;
        //uint32_t adc_readingB = 0;
    
        for(int i = 0; i < NUM_SAMPLES; i++)
        {
            adc_readingR += adc1_get_raw((adc_channel_t)channelR);
            //adc_readingG += adc1_get_raw((adc_channel_t)channelG);
            //adc_readingB += adc1_get_raw((adc_channel_t)channelB);       
        }
            //Commented out is for converting and showing voltage values
            adc_readingR /= NUM_SAMPLES;
            //uint32_t voltageR = esp_adc_cal_raw_to_voltage(adc_readingR, adc_chars);
            //adc_readingG /= MAX_SAMPLES;
            //uint32_t voltageG = esp_adc_cal_raw_to_voltage(adc_readingG, adc_chars);
            //adc_readingB /= MAX_SAMPLES;
            //uint32_t voltageB = esp_adc_cal_raw_to_voltage(adc_readingB, adc_chars);
            //printf("LEDC_Red: %3d\tLEDC_Green: %3d\tLEDC_Blue: %3d\n", adc_readingR, adc_readingG, adc_readingB);
            //printf("LEDC_Red: %d\tVoltage: %dmV\tLEDC_Green: %d\tVoltage: %dmV\tLEDC_Blue: %d\tVoltage: %dmV\n", adc_readingR, voltageR, adc_readingG, voltageG, adc_readingB, voltageB);
            //vTaskDelay(pdMS_TO_TICKS(500)); 
        
            int ch; //0 LEDC_RED, 1 LEDC_GREE, 2 LEDC_BLU 

        ledc_timer_config_t ledc_timer = 
        {
            .duty_resolution = LEDC_RESOLUTION,
            .freq_hz = 5000,
            .speed_mode = LEDC_HS_MODE,
            .timer_num = LEDC_HS_TIMER,
            .clk_cfg = LEDC_CLK,

        };
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] =
        {
            {   //LEDC_RED                
                .channel =      LEDC_RED_CHANNEL,
                .duty =         0,
                .gpio_num =     LEDC_RED_GPIO,
                .speed_mode =   LEDC_HS_MODE,
                .hpoint =       0,
                .timer_sel =    LEDC_HS_TIMER,
            },
            {   //LEDC_BLU                
                .channel =      LEDC_GREE_CHANNEL,
                .duty =         0,
                .gpio_num =     LEDC_GREE_GPIO,
                .speed_mode =   LEDC_HS_MODE,
                .hpoint =       0,
                .timer_sel =    LEDC_HS_TIMER,
            },
            {   //LEDC_GREE                
                .channel =      LEDC_BLU_CHANNEL,
                .duty =         0,
                .gpio_num =     LEDC_BLU_GPIO,
                .speed_mode =   LEDC_HS_MODE,
                .hpoint =       0,
                .timer_sel =    LEDC_HS_TIMER,
            },
        };

        for(ch=0; ch < LEDC_TEST_CH_NUM; ch++)
        {
            ledc_channel_config(&ledc_channel[ch]);
        }

        ledc_fade_func_install(0);

        for(ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
        {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

        //LEDC_RED
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, channelR);
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        
        //LEDC_GREE
        ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, adc_readingG);
        ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        //LEDC_BLU
        ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, adc_readingB);
        ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);
        
        vTaskDelay (1 / portTICK_PERIOD_MS);
        
    }*/
    //vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}

