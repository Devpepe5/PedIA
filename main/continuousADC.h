
typedef struct {
    uint8_t *buffer;                   ///< Number of ADC channels that will be used
    adc_continuous_handle_t *handle; ///< List of configs for each ADC channel that will be used
    uint16_t *numSamples;
    }continuous_args;

typedef struct{
float promedio;
float varianza;
float std_desv; //desviación standard 
float avrg_ener;//enerfía proomedio 
}metricas_estadisticas
