#include "main.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;


void microDelay(int us);  //us : micro second
/*{
	//int t1 = htim2.Instance->CNT;
	//while(htim2.Instance->CNT - t1 > us);

	htim2.Instance->CNT = 0;
	while(htim2.Instance->CNT < us);
}*/

void Trigger();
/*{
	  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
	  microDelay(10);
	  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
	  microDelay(10);
	  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
}*/

double Distance()
{
	  int t0 = 0, t1, t2;

	  htim2.Instance->CNT = 0;
	  Trigger();
	  //wait until Echo High
	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 0)
	  {
		  if(htim2.Instance->CNT > 30000) return -1;		//TimeOut
	  }
	  t1 = htim2.Instance->CNT;

	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1)
	  {
		  if(htim2.Instance->CNT > t1 + 60000) return -1;//wait until Echo Low
	  }
	  t2 = htim2.Instance->CNT;

	  double dist = (t2 - t1) * 0.17;
	  return dist;
}
