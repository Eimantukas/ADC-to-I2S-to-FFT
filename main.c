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
#define I2S_SAMPLE_BITS     I2S_BITS_PER_SAMPLE_16BIT
#define I2S_ADC_UNIT        ADC_UNIT_1                                     //I2S built-in ADC unit
#define I2S_ADC_CHANNEL     ADC1_CHANNEL_3    	                           //I2S built-in ADC channel

#define SAMPLES 1024

static const adc_channel_t channelR = ADC_CHANNEL_3;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

const int NUM_SAMPLES = SAMPLES;
int samplingFrequency =  44000;

static const char *TAG = "main";
float x1[SAMPLES]; //used for generated tone signal in esp
float wind[SAMPLES];
float y_cf[SAMPLES*2];
float* y1_cf = &y_cf[0];

static float real[SAMPLES];
static float imag[SAMPLES];

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
    
    adc1_config_width(width);
    adc1_config_channel_atten(channelR, atten);
    i2s_adc_enable(I2S_NUM);    
    i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);      
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
	
	//dsps_tone_gen_ft32(x1, NUM_SAMPLES, 1.0, 0.16); //internally generated input signal
    
    
    for(int i=0; i < NUM_SAMPLES; i++)
    {
        y_cf[i*2+0]= real[i]*wind[i]; //replace real with x1 for internal input signal
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
    }
}

