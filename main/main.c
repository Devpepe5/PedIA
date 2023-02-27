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
    float promedio[ADCINPUTSNUM],  varianza[ADCINPUTSNUM], desv_std[ADCINPUTSNUM],  energia_promedio[ADCINPUTSNUM], zc[ADCINPUTSNUM];
    float wl[ADCINPUTSNUM],ssc[ADCINPUTSNUM];


//  float *ppromedio, *pzc, *pdesv_std, *pvarianza, *p_energia_promedio  
/*
    pzc = &zc;
    ppromedio = &promedio;
    pvarianza = &varianza;
    pdesv_std = &desv_std;
    p_energia_promedio = &energia_promedio;
*/


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
    metrics.promedio=promedio;
    metrics.varianza=varianza;
    metrics.desv_std=desv_std; //desviación standard 
    metrics.energia_promedio=energia_promedio; 
    metrics.zc=zc;
    metrics.ssc= ssc;
    metrics.wl=wl;

    metricas_estadisticas *pmetrics =&metrics; // estrucutra para almacenar resultados procesados
    
    static continuous_args  ADC_args_struct;
    ADC_args_struct.buffer= bufferADC;
    ADC_args_struct.handle= phandle; 
    ADC_args_struct.numSamples = pnumSamples;
    ADC_args_struct.metrics=pmetrics;
    //ADC_args_struct.ssc=ssc;
    //ADC_args_struct.voltsBuffer=vectorPruebaZC; //Asiganación de prueba para comprobar correcta aplicación de función zerocross()
    ADC_args_struct.voltsBuffer=voltsBuffer;
    continuous_args *pADC_args=&ADC_args_struct;
    //cruces_por0(pADC_args, pmetrics);
    zerocross(&ADC_args_struct);

    //printf("puntero a enviar: bufferADC: %p  handle: %p numero de muestras  %p numero de muestra pero  con &: %p \n", ADC_args_struct.buffer, ADC_args_struct.handle, ADC_args_struct.numSamples , &numSamples);// chequeo de punteros 
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    //printf("zc calculado por el micro= %.3f zc calculado por matlab %.3f ", zc, valorZCEsperado);
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
    }
    
    vTaskDelay(1);
    }
}



void procesado(void *parametros){
    //variabbles de entrada
    continuous_args *args = (continuous_args*)parametros ;
    uint8_t *bufferADC=(args->buffer);
    adc_continuous_handle_t *handle =(args->handle);
    uint32_t *num_Samples = (args->numSamples);
    float *voltsBuffer= (args->voltsBuffer);
    //variables para procesado 
    float mean[ADCINPUTSNUM];
    float zcb[ADCINPUTSNUM];
    float rms[ADCINPUTSNUM];
    int ssc[ADCINPUTSNUM];
    float diffvolts[(*num_Samples)-1];
    float sumdiffvolts[ADCINPUTSNUM]; //para hacer la sumatoria 
     for(int i=0;i<ADCINPUTSNUM;i++){
        mean[i] = 0;
        zcb[i] =  0;
        rms[i] = 0;
        ssc[i]=0;
        //diffvolts[i]=0;
        sumdiffvolts[i]=0;
    }
    //inicializa las varibles en ceros para 
    
    float tmp2;  // valor anterior de work
    float work=0; //valor auxiliar
    float work_tmp; //; //valor actual temporal
    float zc;
    float *zcOUT=args->metrics->zc; //puntero el valor de salida ZC 
    float *meanOUT=args-> metrics->promedio;  
    float *wlOUT=args-> metrics->wl;
    float *rmsOUT=args-> metrics->rms;
    //float *x= args->voltsBuffer;
    for (uint32_t i=0;i<*num_Samples;i += SOC_ADC_DIGI_RESULT_BYTES )// que haga la suma de cuantos bytes va avanzando "creo"
    {
                    adc_digi_output_data_t *p = (void*)&bufferADC[i];
                    uint32_t chan_num = ADC_GET_CHANNEL(p);
                    uint32_t data = ADC_GET_DATA(p);
                    //printf("canal: %lu valor: %lu \n",chan_num, data);  //  muestreo de valores
                    voltsBuffer[i]= data*(MAX_ADC_mVOLT/DMAX);  
                    mean[i % ADCINPUTSNUM]+=voltsBuffer[i];// sumatoria para calcular el promedio 
                    //printf("canal: %lu valor: %.3f \n",chan_num, voltsBuffer[i]);  //muestreo de valores reales //
                    rms[i % ADCINPUTSNUM]+= (voltsBuffer[i]*voltsBuffer[i]);
                    
                    if (i>ADCINPUTSNUM){
                        diffvolts[i % ADCINPUTSNUM] = (voltsBuffer[i]-voltsBuffer[i-ADCINPUTSNUM]); // si solo hago i-1 estaría restando el valor de otro canal
                    }
                    sumdiffvolts[i % ADCINPUTSNUM] += fabs(diffvolts[i]);
                    //algoritmo para ZC
                    work_tmp = voltsBuffer[i];
                    if (work_tmp < 0.0) {
                    work_tmp = -1.0;
                    } else {
                            work_tmp = (work_tmp > 0.0); //se estaría asignando el valor 1 a partir de resultado de la comparación
                            }
                    //algoritmo para SSC
                    if (diffvolts[i] < 0.0) {
                    diffvolts[i] = -1.0;
                    } else {
                            diffvolts[i] = (diffvolts[i] > 0.0); //se estaría asignando el valor 1 a partir de resultado de la comparación
                            }   
                    if(i>ADCINPUTSNUM){
                        if (diffvolts[i]!=diffvolts[i-ADCINPUTSNUM])
                        ssc[i % ADCINPUTSNUM]+=1;
                    }      
    //este  codigo genera un -1 o 1 a os valores que son menores a cero  y mayores aceros respectivamente
    //al hacer la resta de valores que tuvieron un cambio anterior 
    //sumara a zcb el numero 2, un numero distinto a cero
    tmp2 = work;
    work = work_tmp;
    
    //de esta manera se almacena el valor por canal 
    zcb[i % ADCINPUTSNUM] += fabs(work_tmp - tmp2);
    
    
    //zc += zcb[i]; //sumatoria de cada vez que cruzo por cero multiplicada por 2 

  }
  

  //return 0.5 * work_tmp / 100.0; // los return estan prohibidos
  for (int k=0 ; k<ADCINPUTSNUM ; k++){
  *(zcOUT+k)= 0.5 * zcb[k] / 100.0;
  *(meanOUT+k)= mean[k]/(*num_Samples);
  *(rmsOUT+k)= rms[k]/(*num_Samples);
  *(wlOUT+k)=sumdiffvolts[k]/(*num_Samples);

    }
    }
    



//Funciones





//la diferencia entre las tareas "tasks" y las funciones es que las funciones tienesn 

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

void SSC(continuous_args *structin, metricas_estadisticas *structout){

}

