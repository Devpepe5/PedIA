

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
