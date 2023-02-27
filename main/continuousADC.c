

/*codigo de las funciones */


#include "continuousADC.h"
//cambiamos la función para poder correrla en un ambiente RTOS  no returns


void zerocross(continuous_args *dataStruct){ 

  float zcb[100];
  float tmp2;  // valor anterior de work
  float work; //valor auxiliar
  float work_tmp; //; //valor actual temporal
  float *zcOUT=dataStruct->metrics->zc; //puntero el valor de salida ZC   
  float *x= dataStruct->voltsBuffer;
  //if (!isInitialized_zerocross) {
  //  zerocross_initialize();
  //}
  int k;
  work = 0.0;
  for (k = 0; k < 100; k++) {
    work_tmp = x[k];
      if (work_tmp < 0.0) {
      work_tmp = -1.0;
    } else {
      work_tmp = (work_tmp > 0.0); //se estaría asignando el valor 1 a partir de resultado de la comparación
    }
    //este  codigo genera un -1 o 1 a os valores que son menores a cero  y mayores aceros respectivamente
    //al hacer la resta de valores que tuvieron un cambio anterior 
    //sumara a zcb el numero 2, un numero distinto a cero
    tmp2 = work;
    work = work_tmp;
    zcb[k] = fabs(work_tmp - tmp2);
  }
  work_tmp = zcb[0];
  for (k = 0; k < 99; k++) {
    work_tmp += zcb[k + 1]; //sumamos todos los valores que tengamos de cambios
  }
  //return 0.5 * work_tmp / 100.0; // los return estan prohibidos
  *zcOUT= 0.5 * work_tmp / 100.0;

}


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


  void RMS(continuous_args *structin, metricas_estadisticas *structout) {// root mean square
    float *volts = structin->voltsBuffer;
    uint32_t *numSamples=structin->numSamples;
    float rms=0, *prms;
    prms=structout->rms;
    for (uint32_t i=0; i<*numSamples;i++) {
        rms+=((volts[i])*(volts[i])); //sumatoria x^2
    }
        rms=rms/(*numSamples);
        rms=sqrt(rms);
        *prms=rms;
    }


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


