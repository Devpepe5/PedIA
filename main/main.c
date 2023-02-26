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
#include "continuousADC.h"

#include <math.h>
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
#define BUFFER_LEN_                 (CONVERSION_FRAME_SIZE)// SI Y SOLO SI BUFFER%4 =0
#define MAX_ADC_mVOLT               (3300) //maximo valor de lectura en milivolts
#define DMAX                        (4095)  // valor maximo digital 2^12
//lista de canales a usar
static adc_channel_t channel_list[ADCINPUTSNUM]={ADC_CHANNEL_6, ADC_CHANNEL_7};

float vectorPruebaZC[100]= {287, 378, -401, 385,	104, -430, -249, 19, 430, 437, -370, 443, 429, -43, 272, -386, -106, 388, 264, 431, 128, -492, 321, 406, 151, 230, 215, -136, 127, -357, 178, -496, -251, -482, -431, 295, 167, -211, 422, -494, -89, -146, 238, 267, -341, -38, -82, 118, 181, 227, -252, 152, 127, -365, -409, -30, 432, -188, 57, -304, 223, -273, -22, 171, 363, 431, 19, -389, -379, -270, 313, -274, 286, -284, 401, -178, -331, -277, 88, -55, -176, 303, 57, 22, 389, -242, 229, 226, -148, 40, -452, -474, 3, 251, 406, -398, 41, -59, -516, -191};
float valorZCEsperado=.5350;

/*para su correcta aplicación se tiene que usar un ADC en modo pseudobipolar,
 o restar el valor de offset a el resultado
*/ 
 
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

void adc_continuous_init(adc_channel_t *channel_list, uint8_t canales_numero, adc_continuous_handle_t *handle_salida );
void procesado(void *parametros);
void lectura_adc(void *parametros);

void promedio(continuous_args *structin, metricas_estadisticas *structout);  // mean average value MAV 
void RMS(continuous_args *structin, metricas_estadisticas *structout);// root mean square
void longitud_de_onda(continuous_args *structin, metricas_estadisticas *structout); //Wave length WL
void cruces_por0(continuous_args *structin, metricas_estadisticas *structout); //zero crossing
void SSC(continuous_args *structin, metricas_estadisticas *structout);//Slope Scope Change (SSC) "Cambio de alcance de pendiente". según el traductor 


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
     uint8_t bufferADC[BUFFER_LEN_] = {0};// datos crudos 
     float voltsBuffer[BUFFER_LEN_] = {0}; //datos procesados, en volts
     uint32_t  numSamples=0,*pnumSamples;
     pnumSamples =&numSamples;
    //metricas estadisticas
    float promedio, *ppromedio, varianza, *pvarianza, desv_std, *pdesv_std, energia_promedio, *p_energia_promedio;
    float zc, *pzc; 
    pzc = &zc;
    ppromedio = &promedio;
    pvarianza = &varianza;
    pdesv_std = &desv_std;
    p_energia_promedio = &energia_promedio;



    //uint16_t avrgBuffer=0;
    //muestras tomadas
    //int procesosNum=2;
    //uint8_t ADC_BITWIDTH=12;
    memset(bufferADC, 0xcc, BUFFER_LEN_);
    //memset(voltsBuffer,0xcc, BUFFER_LEN_);
     static adc_continuous_handle_t handle = NULL;
     adc_continuous_handle_t *phandle = &handle;
    
    
    adc_continuous_init(channel_list, sizeof(channel_list) / sizeof(adc_channel_t), phandle);
    
    static metricas_estadisticas metrics;
    //almaceno la dirección de las variables
    metrics.promedio=ppromedio;
    metrics.varianza=pvarianza;
    metrics.desv_std=pdesv_std; //desviación standard 
    metrics.energia_promedio=p_energia_promedio; 
    metrics.zc=pzc;
    metricas_estadisticas *pmetrics =&metrics; // estrucutra para almacenar resultados procesados
    
    static continuous_args  ADC_args_struct;
    ADC_args_struct.buffer= bufferADC;
    ADC_args_struct.handle= phandle; 
    ADC_args_struct.numSamples = pnumSamples;
    ADC_args_struct.metrics=pmetrics;
    ADC_args_struct.voltsBuffer=vectorPruebaZC; //Asiganación de prueba para comprobar correcta aplicación de función zerocross()
    //ADC_args_struct.voltsBuffer=voltsBuffer;
    continuous_args *pADC_args=&ADC_args_struct;
    //cruces_por0(pADC_args, pmetrics);
    zerocross(&ADC_args_struct);

    //printf("puntero a enviar: bufferADC: %p  handle: %p numero de muestras  %p numero de muestra pero  con &: %p \n", ADC_args_struct.buffer, ADC_args_struct.handle, ADC_args_struct.numSamples , &numSamples);// chequeo de punteros 
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    
    printf("zc calculado por el micro= %.3f zc calculado por matlab %.3f ", zc, valorZCEsperado);
    vTaskDelay(10*100/portTICK_PERIOD_MS);
    xTaskCreate(lectura_adc, "Lectura continua ADC", 1024*4,(void*)pADC_args , configMAX_PRIORITIES-1,NULL);
}
//    ESP_ERROR_CHECK(adc_continuous_stop(handle));
//    ESP_ERROR_CHECK(adc_continuous_deinit(handle));



//void *ADC_args[5]={&bufferADC, &handle, &numSamples};// copia de la declaracion para tener referencia
void lectura_adc(void *parametros)
{   
    continuous_args *pDatos=(continuous_args*)parametros;
    
    uint8_t *bufferADC=(pDatos->buffer);
    adc_continuous_handle_t *handle =(pDatos->handle);
    uint32_t *num_Samples = (pDatos->numSamples);
    float *voltsBuffer= (pDatos->voltsBuffer);
    //printf("punteros recibidos buffer : %p, puntero handle: %p, puntero del numero de muestras: %p \n", ADC_args[0], ADC_args[1], ADC_args[2]);
     printf("punteros recibidos buffer : %p, puntero handle: %p, puntero del numero de muestras: %p \n", bufferADC, handle, (pDatos->numSamples));
    
    esp_err_t ret;  
    
    //printf("inicia la lectura numero de muestras inicial: %lu \n", numSamples);

    while(1)
    {
    ret = adc_continuous_read(*handle, bufferADC, BUFFER_LEN_,num_Samples, 0);
    if ((ret)==ESP_OK)
    {
    printf("Lectura exitosa");
    for (uint32_t i=0;i<*num_Samples;i += SOC_ADC_DIGI_RESULT_BYTES )// que haga la suma de cuantos bytes va avanzando "creo"
    {
        adc_digi_output_data_t *p = (void*)&bufferADC[i];
                    uint32_t chan_num = ADC_GET_CHANNEL(p);
                    uint32_t data = ADC_GET_DATA(p);
                    printf("canal: %lu valor: %lu \n",chan_num, data);//muestreo de valores
                    voltsBuffer[i]= data*(MAX_ADC_mVOLT/DMAX);
                    printf("canal: %lu valor: %.3f \n",chan_num, voltsBuffer[i]);//muestreo de valores reales //   
    }
                    
    }
    
    vTaskDelay(1);
    }
}



void procesado(void *parametros){
    
}


//Funciones
void adc_continuous_init(adc_channel_t *channel_list, uint8_t canales_numero, adc_continuous_handle_t *handle_salida )
{

    
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
    // a Través de un bucle guardamos todos los valores que serán similares
    for (uint16_t i= 0 ; i<canales_numero ; i++){
    adc_patron[i].atten=ADC_ATTEN_DB_11;      ///< Attenuation of this ADC channel
    adc_patron[i].channel=channel_list[i]&0x7;    ///< ADC channel
    adc_patron[i].unit=ADC_UNIT_1;       ///< ADC unit
    adc_patron[i].bit_width=ADC_BITWIDTH_12; //profundidad maxima
    }
    digitalConfigADC.adc_pattern=adc_patron;// 
    ESP_ERROR_CHECK(adc_continuous_config(handle, &digitalConfigADC));   
    *handle_salida=handle;
    printf("inicialización exitosa\n");

}





//la diferencia entre las tareas "tasks" y las funciones es que las funciones tienesn 
void RMS(continuous_args *structin, metricas_estadisticas *structout){// root mean square
    float *volts = structin->voltsBuffer;
    uint32_t *numSamples=structin->numSamples;
    float rms=0, *prms;
    prms=structout->rms;
    for (uint32_t i=0; i<*numSamples;i++) {
        rms+=((volts[i])*(volts[i]));
    }
        rms=rms/(*numSamples);
        rms=sqrt(rms);
        *prms=rms;
    }
void longitud_de_onda(continuous_args *structin, metricas_estadisticas *structout){ //Wave length WL

} 
void cruces_por0(continuous_args *structin, metricas_estadisticas *structout){ //zero crossing ZC
//float *volts = structin->voltsBuffer;
float zc=0, *pzc;
pzc=structout->zc;
//zerocross(vectorPruebaZC);
*pzc=zc;
printf("el numero de cruces por cero calculado es: %.3f valor calculado por matlab: %.3f  ",zc ,valorZCEsperado);
} 
//Slope Scope Change (SSC) "Cambio de alcance de pendiente". según el traductor 
void promedio(continuous_args *structin, metricas_estadisticas *structout)  // mean average value MAV
{   
    uint32_t *numSamples=structin->numSamples; 
    float promedio=0, *ppromedio;
    ppromedio=structout->promedio;
    uint8_t *bufferADC=structin->buffer;
    for (int i=0; i<(*numSamples);i++)
    {
        promedio=promedio + bufferADC[i];
    }
    promedio=promedio/(*numSamples);
    printf("promedio calculado: %.3f ",promedio);
    *ppromedio=promedio;    
}
void SSC(continuous_args *structin, metricas_estadisticas *structout){

}

