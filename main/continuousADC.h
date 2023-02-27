

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




#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"

//definicioón de tipos de variable
typedef struct{
float *promedio; //calculado //comprobado
float *varianza; //no necesario calcularlo por el momento 
float *desv_std; //desviación standard   //  no necesario calcular 
float *energia_promedio;//energía proomedio //no necesario calcular por el momento  
float *zc;  //calculado //comprobado
float *rms; //calculado 
float *ssc; //calculado
float *wl;
}metricas_estadisticas;


typedef struct {
    uint8_t *buffer;                   ///< buffer de lecutra de datos crudos;
    float *voltsBuffer;                 //valor digital a real en voltaje
    adc_continuous_handle_t *handle; ///< List of configs for each ADC channel that will be used
    uint32_t *numSamples;
    metricas_estadisticas *metrics; // metrics es una subestructura de 
    }continuous_args;

//definición de funciones
void zerocross(continuous_args *dataStruct);
void adc_continuous_init(adc_channel_t *channel_list, uint8_t canales_numero, adc_continuous_handle_t *handle_salida );
void promedio(continuous_args *structin, metricas_estadisticas *structout);
void RMS(continuous_args *structin, metricas_estadisticas *structout);



