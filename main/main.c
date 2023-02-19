/*Cambios necesarios= "El controlador adc heredado está en desuso, migre para usar esp_adc/adc_oneshot.h y esp_adc/adc_continuous.h para los controladores de modo único y modo continuo, respectivamente"


*/




#include <stdio.h>
#include <string.h>
//#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//#include <freertos/queue.h>
//#include "driver/adc.h"
#include <freertos/semphr.h>
//#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
//#include <freertos/timers.h>
//#include "driver/timer.h"
#define RATE_HZ_1a1000                  (200)

#define MUESTREOTICKSTOWAIT             (RATE_HZ_1a1000/1000)
#define ADC_BIT_WIDTH                   (12)
//static TimerHandle_t muestreo// 
//static hw_timer_t *timer = NULL; // handler del timer

//static float lecturas               [1000]
#define ADCINPUTSNUM                    (2)
#define SAMPLERATE                  (ADCINPUTSNUM*10*1000) //numero de entradas por ratesample unitario
#define ADC_GET_CHANNEL(pDato)      ((pDato)->type2.channel)
#define ADC_GET_DATA(pDato)         ((pDato)->type2.data)
#define CONVERSION_FRAME_SIZE       (100) // número de muestras que se van a tomar por lectura
#define MAX_BUFFER_SIZE             (10*(CONVERSION_FRAME_SIZE))
#define BUFFER_LEN_                      (CONVERSION_FRAME_SIZE)// SI Y SOLO SI BUFFER%4 =0
//lista de canales a usar
static adc_channel_t channel_list[ADCINPUTSNUM]={ADC_CHANNEL_6, ADC_CHANNEL_7};

 
//variables globales

/**/
/* ESP-IDF  tiene API´s para el uso del ADC en modo continuo
void lectura_i2s_ADC(void *parameters)// puedo usar un puntero al buffer 
{
   // uint16_t *buffer;
   // buffer=bufferADC;
    
    int i2s_read_len=I2S_READ_LEN;
    i2s_adc_enable(I2S_NUM_0);
    i2s_read(I2S_NUM_0,(void *)bufferADC, sizeof(bufferADC), &muestrasTomadasNum, portMAX_DELAY);
    xSemaphoreGive(ADCdone);//permitimos que se ejecten otros procesos
    i2s_adc_disable(I2S_NUM_0);
    
    xSemaphoreGive(ADCdone);
    vTaskDelete(NULL);
    
}
*/

static void adc_continuous_init(adc_channel_t *channel_list, uint8_t canales_numero, adc_continuous_handle_t *handle_salida );
void procesado(void *parametros);
void lectura_adc(void *parametros);
/*EL uso de callback de timer se reserva para otro uso 
void CallbackMuestreo(TimerHandle_t xTimer)
{
    if((uint32_t)pvTimerGetTimerID(xTimer)==0){// un solo timer 
        printf("");
    }
    
}
*/

void app_main(void)
{   
    
    
    //SemaphoreHandle_t ADCdone=NULL;
    static uint8_t bufferADC[BUFFER_LEN_] = {0};
    esp_err_t ret;
    volatile uint32_t numSamples=0;



    //uint16_t avrgBuffer=0;
    //muestras tomadas
    //int procesosNum=2;
    //uint8_t ADC_BITWIDTH=12;
    memset(bufferADC, 0xcc, BUFFER_LEN_);
    adc_continuous_handle_t handle = NULL;
    void *ADC_args[5]={&bufferADC, &handle, &numSamples}; //arreglo de argumentos para poder acceder a cada dato que necesitaremos 
    adc_continuous_init(channel_list, sizeof(channel_list) / sizeof(adc_channel_t), &handle );
    

   
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    
     //   ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //lectura 
    
    //ESP_ERROR_CHECK(adc_continuous_read(handle, bufferADC, BUFFER_LEN_, &numSamples, 0));
    //vTaskDelay(10/portTICK_PERIOD_MS);
    
    vTaskDelay(11);
    
    printf("fin de la lectura\n");
    
    

  //  ADCdone=xSemaphoreCreateCounting(procesosNum,0);
    

/*            xTaskCreate(promediarBuffer,
                    "promedio del buffer",
                    1024,
                    (void *)avrgBuffer,
                    configMAX_PRIORITIES-2,
                    NULL);
 

            xTaskCreate(mostrarBuffer,
                    "mostrar buffer en pantalla",
                    1024,
                    (void *)bufferADC,
                    configMAX_PRIORITIES-2,
                    NULL);

*/
xTaskCreate(lectura_adc, "Lectura continua ADC", 1024*2,(void*)ADC_args,1,NULL);
}
//    ESP_ERROR_CHECK(adc_continuous_stop(handle));
//    ESP_ERROR_CHECK(adc_continuous_deinit(handle));

//Funciones
static void adc_continuous_init(adc_channel_t *channel_list, uint8_t canales_numero, adc_continuous_handle_t *handle_salida )
{
int inputs_num=ADCINPUTSNUM;
    
    static adc_continuous_handle_t handle=NULL;//inicio el handler
    adc_continuous_handle_cfg_t configHandleADCContinuo= { 
        .max_store_buf_size = MAX_BUFFER_SIZE,
        .conv_frame_size = CONVERSION_FRAME_SIZE,
    }; //declaradción del    la configuración del handler
    ESP_ERROR_CHECK(adc_continuous_new_handle(&configHandleADCContinuo, &handle));
    adc_continuous_config_t digitalConfigADC={
        .sample_freq_hz= SAMPLERATE,                /*!< The expected ADC sampling frequency in Hz. Please refer to `soc/soc_caps.h` to know available sampling frequency range*/
        .conv_mode=ADC_CONV_SINGLE_UNIT_1,      ///< ADC DMA conversion mode, see `adc_digi_convert_mode_t`.
        .format=ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num=ADCINPUTSNUM,
    };   
    
    adc_digi_pattern_config_t adc_patron[ADCINPUTSNUM];
    //uint8_t bitwidth=12;
    // a Través de un bucle gradamos todos los valores que serán similares
    for (uint16_t i= 0 ; i<canales_numero ; i++){
    adc_patron[i].atten=ADC_ATTEN_DB_11;      ///< Attenuation of this ADC channel
    adc_patron[i].channel=channel_list[i]&0x7;    ///< ADC channel
    adc_patron[i].unit=ADC_UNIT_1;       ///< ADC unit
    adc_patron[i].bit_width=ADC_BITWIDTH_12; //profundidad maxima
    }
    digitalConfigADC.adc_pattern=adc_patron;// 
    
    ESP_ERROR_CHECK(adc_continuous_config(handle, &digitalConfigADC));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    *handle_salida=handle;
}

//void *ADC_args[5]={&bufferADC, &handle, &numSamples};// copia de la declaracion para tener referencia
void lectura_adc(void *parametros)
{   
    void *ADC_args[5]={(void *)parametros};
    static uint8_t bufferADC[BUFFER_LEN_]={};
    *bufferADC=(uint8_t *)ADC_args[0];
    adc_continuous_handle_t handle*;
    handle=(adc_continuous_handle_t *)ADC_args[1];
    volatile uint32_t *numSamples;
    numSamples= (uint32_t *)ADC_args[2];

    esp_err_t ret;
    
    printf("inicia la lectura\n");
    while(1)
    {
    ret = adc_continuous_read(handle, bufferADC, BUFFER_LEN_, &numSamples, 0);
    if ((ret)==ESP_OK)
    {
    printf("Lectura exitosa");
    for (uint32_t i=0;i=num_samples;i += SOC_ADC_DIGI_RESULT_BYTES )// que haga la suma de cuantos bytes va avanzando "creo"
    {
        adc_digi_output_data_t *p = (void*)&bufferADC[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    printf("");
    }
    }
    }
}
/*
void procesado(void *parametros)
{
    adc_digi_output_data_t *pBuffer = (void*)&bufferADC[0];
    uint32_t canal[ADCINPUTSNUM]={0};
    canal[0] = ADC_GET_CHANNEL(pBuffer);
    pBuffer = (void*)&bufferADC[1];
    canal[1] = ADC_GET_CHANNEL(pBuffer);
    for(int i=0; i<numSamples; i+= SOC_ADC_DIGI_RESULT_BYTES )
    {
        pBuffer = (void*)&bufferADC[i];
        uint32_t dato=ADC_GET_DATA(pBuffer);    
    }
     
}
*/