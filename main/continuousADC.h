

#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
//definicioón de tipos de variable
typedef struct{
float *promedio;
float *varianza;
float *desv_std; //desviación standard 
float *energia_promedio;//enerfía proomedio 
float *zc; 
float *rms;
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




