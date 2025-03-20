/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <string.h>
//#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uSonic INTR----------------------------------------------------------------------------------
//int f_1 = 0 , f_2 = 0, f_r1 =0 , f_r2 = 0, f_l1 =0 , f_l2 = 0;
//int b_1 = 0 , b_2 = 0, b_r1 =0 , b_r2 = 0, b_l1 =0 , b_l2 = 0;
//double dist = 0.0, f_rdist = 0.0, f_ldist = 0.0;
//double b_dist = 0.0, stop_dist = 0.0;

uint32_t f_1 = 0, f_2 = 0, f_r1 = 0, f_r2 = 0, f_l1 = 0, f_l2 = 0;
uint32_t b_1 = 0, b_2 = 0, b_r1 = 0, b_r2 = 0, b_l1 = 0, b_l2 = 0;
double dist = 0.0, f_rdist = 0.0, f_ldist = 0.0;
double b_dist = 0.0, b_rdist = 0.0, b_ldist = 0.0;


// 명령 종류 ?���????
typedef enum {
    CMD_NONE,
    CMD_FORWARD,
    CMD_RIGHT,
    CMD_BACKWARD,
    CMD_LEFT
} CommandType;

#define BUF_SIZE 100
#define CMD_QUEUE_SIZE    5
#define SAFE_DISTANCE 10.0

char dum1, dum2;
char buf1[BUF_SIZE], buf2[BUF_SIZE]; // DMA buffer for UART1, UART2

volatile int t1 = 0, t2 = 0;
volatile CommandType currentCommand = CMD_NONE;
volatile uint8_t commandActive = 0;
volatile uint8_t commandStep = 0;
volatile uint32_t commandStartTime = 0;
volatile uint8_t mode = 255; // 255: 미설?��, 1: ?��?��주행, 0: ?���???? 주행
volatile uint8_t onOff = 0;  // 0: OFF, 1: ON

// 🛠 TIM Input Capture 시작 함수 (초음파 센서 측정 시작)
void Start_Ultrasonic_Measurement() {
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);  // 전방 센서
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);  // 전방 우측 센서
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);  // 전방 좌측 센서
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);  // 후방 센서
}

void ultrasonic_trigger_setup() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 전방 센서 // Trig
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 전방 우측 센서 // Trig2
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 전방 좌측 센서 // Trig3
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 후방 센서 // Trig1
}


//// 🚗 자율 주행 제어 함수
//void auto_drive() {
//    if (dist > SAFE_DISTANCE && f_rdist > SAFE_DISTANCE && f_ldist > SAFE_DISTANCE) {
//        motor_forward();
//    }
//    else if (dist <= SAFE_DISTANCE) {
//        motor_stop(); // 정지
//        if (f_rdist > f_ldist) {
//            motor_right();
//        } else {
//            motor_left();
//        }
//    }
//}

int stuck_counter = 0;






    void auto_motor(){
        float lane_width = f_rdist + f_ldist; // 현재 도로 폭 계산

        if (dist < 10.0 && f_rdist < 10.0 && f_ldist < 10.0) {
            stuck_counter++;

            if (stuck_counter > 2) {
                // 🚗 **후진 후 90도 회전**
                motor_backward();
                HAL_Delay(300);

                if (f_rdist > f_ldist) {
                    motor_right(); // 90도 회전
                } else {
                    motor_left(); // 90도 회전
                }
                HAL_Delay(200);

                // 🚗 **90도 회전 후 일정 거리 전진하여 같은 위치로 돌아가지 않음**
                motor_forward();
                HAL_Delay(300);

                stuck_counter = 0; // 탈출 후 카운터 리셋
            }
        }

        if(mode == 0)
        {
        	commandActive = 0;
        	currentCommand = CMD_NONE;
        }
        if (dist < 10.0) {
            // 장애물 감지 → 후진 후 좌우 비교하여 회피
            motor_backward();
            HAL_Delay(300);

            if (f_rdist > f_ldist) {
                motor_right();
            } else {
                motor_left();
            }
            HAL_Delay(200);

        } else if (dist < 15.0) {
            // 장애물 10~15cm → 더 넓은 쪽으로 회피
            if (f_rdist > f_ldist) {
                motor_right();
            } else {
                motor_left();
            }
            HAL_Delay(200);

        } else {
            // 🚗 **중앙 유지 주행 로직**
            float balance = f_rdist - f_ldist; // 좌우 차이 계산

            if (lane_width < 37.0) {
                // 좁은 구간 (35cm 근처)
                if (balance > 5.0) {
                    motor_right();
                } else if (balance < -5.0) {
                    motor_left();
                } else {
                    motor_forward();
                }
                HAL_Delay(100);

            } else if (lane_width > 43.0) {
                // 넓은 구간 (45cm 근처)
                if (balance > 8.0) {
                    motor_right();
                } else if (balance < -8.0) {
                    motor_left();
                } else {
                    motor_forward();
                }
                HAL_Delay(100);
            } else {
                motor_forward();
            }
        }
        HAL_Delay(100);
    }



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {  // TIM1 사용
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1:  // 전방 초음파 센서
                if (HAL_GPIO_ReadPin(Echof_GPIO_Port, Echof_Pin) == GPIO_PIN_SET) {
                    f_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                } else {
                    f_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                    dist = (f_2 - f_1) * 0.017;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:  // 전방 우측 초음파 센서
                if (HAL_GPIO_ReadPin(Echo_r_GPIO_Port, Echo_r_Pin) == GPIO_PIN_SET) {
                    f_r1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                } else {
                    f_r2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                    f_rdist = (f_r2 - f_r1) * 0.017;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:  // 전방 좌측 초음파 센서
                if (HAL_GPIO_ReadPin(Echof_l_GPIO_Port, Echof_l_Pin) == GPIO_PIN_SET) {
                    f_l1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                } else {
                    f_l2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                    f_ldist = (f_l2 - f_l1) * 0.017;
                }
                break;
            case HAL_TIM_ACTIVE_CHANNEL_4:  // 후방 초음파 센서
                if (HAL_GPIO_ReadPin(Echob_GPIO_Port, Echob_Pin) == GPIO_PIN_SET) {
                    b_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                } else {
                    b_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                    b_dist = (b_2 - b_1) * 0.017;
                }
                break;
        }
    }
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    switch (GPIO_Pin)
//    {
//        case Echof_Pin:
//            if (HAL_GPIO_ReadPin(Echof_GPIO_Port, Echof_Pin) == 1)
//            {
//                f_1 = htim2.Instance->CNT;
//            }
//            else // Falling Edge
//            {
//                f_2 = htim2.Instance->CNT;
//                dist = (f_2 - f_1) * 0.17;
//                if (dist > 2000) dist = 1800;
//            }
//            break;
//
//        case Echof_r_Pin:
//            if (HAL_GPIO_ReadPin(Echof_r_GPIO_Port, Echof_r_Pin) == 1)
//            {
//                f_r1 = htim2.Instance->CNT;
//            }
//            else // Falling Edge
//            {
//                f_r2 = htim2.Instance->CNT;
//                f_rdist = (f_r2 - f_r1) * 0.17;
//                if (f_rdist > 2000) dist = 1800;
//            }
//            break;
//
//        case Echof_l_Pin:
//            if (HAL_GPIO_ReadPin(Echof_l_GPIO_Port, Echof_l_Pin) == 1)
//            {
//               f_l1 = htim2.Instance->CNT;
//            }
//            else
//            {
//               f_l2 = htim2.Instance->CNT;
//               f_ldist = (f_l2 - f_l1) * 0.17;
//               if (f_ldist > 2000) dist = 1800;
//            }
//            break;
//        case Echob_Pin:
//            if (HAL_GPIO_ReadPin(Echob_GPIO_Port, Echob_Pin) == 1)
//            {
//                b_1 = htim2.Instance->CNT;
//            }
//            else // Falling Edge
//            {
//                b_2 = htim2.Instance->CNT;
//                b_dist = (b_2 - b_1) * 0.17;
//                if (b_dist > 2000) dist = 1800;
//            }
//            break;
//
//        case Echo_up_Pin:
//            if (HAL_GPIO_ReadPin(Echo_up_GPIO_Port, Echo_up_Pin) == 1)
//            {
//                b_l1 = htim2.Instance->CNT;
//            }
//            else
//            {
//                b_l2 = htim2.Instance->CNT;
//                stop_dist = (b_l2 - b_l1) * 0.17;
//                if (stop_dist > 2000) dist = 1800;
//            }
//            break;
//
//       default:
//           break;
//    }
//}
//------------------------------------------------------------------------------------------------------------

// 명령 큐 (원형 큐)
char cmdQueue[CMD_QUEUE_SIZE][BUF_SIZE];
volatile int queueHead = 0, queueTail = 0, queueCount = 0;

// 큐에 명령 추가 (인터럽트 문맥)
void EnqueueCommand(const char* cmd)
{
    if(queueCount < CMD_QUEUE_SIZE)
    {
        strcpy(cmdQueue[queueTail], cmd);
        queueTail = (queueTail + 1) % CMD_QUEUE_SIZE;
        queueCount++;
    }
    // 큐가 꽉 찼을 경우 예외 처리 필요
}

// 큐에서 명령 꺼내기 (메인 루프에서 호출)
int DequeueCommand(char* dest)
{
    if(queueCount > 0)
    {
        strcpy(dest, cmdQueue[queueHead]);
        queueHead = (queueHead + 1) % CMD_QUEUE_SIZE;
        queueCount--;
        return 1;
    }
    return 0;
}


// LCD에 항상 모드 표시
void LCD_UpdateModeStatus(void) {
    LCD_command(0xC0); // LCD의 두 번째 줄
    if (mode == 255) {
        LCD_Print("[Harman]Semicon ");
    } else if (mode == 1) {
        LCD_Print("Mode: Auto      ");
    } else {
        LCD_Print("Mode: Manual    ");
    }
}


// 문자열 앞뒤 공백 제거 함수
char *Trim(const char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    int tail = strlen(s);
    while (tail > 0 && (s[tail - 1] == ' ' || s[tail - 1] == '\t' || s[tail - 1] == '\r' || s[tail - 1] == '\n')) tail--;

    char *dest = (char *)malloc(tail + 1);
    if (!dest) return NULL;

    strncpy(dest, s, tail);
    dest[tail] = '\0';
    return dest;
}


// 블루투스 명령 처리 함수
void ProcessBluetoothCommand(char input) {
    if (input == 'C' || input == 'c') {
        mode = (input == 'C') ? 1 : 0;
        printf("Mode set to: %s\n\r", (mode == 1) ? "Auto" : "Manual");

        // 1번째 줄에 메시지 출력
//        LCD_Print_With_Clear((mode == 1) ? "Mode: Auto" : "Mode: Manual");
        // 2번째 줄에 현재 모드 유지
        LCD_UpdateModeStatus();
    }
    else if ((input == 'D' || input == 'd') && mode != 255) {
        onOff = (input == 'D') ? 1 : 0;
        printf("System %s\n\r", (onOff == 1) ? "ON" : "OFF");

        // 1번째 줄에 메시지 출력
        LCD_Print_With_Clear((onOff == 1) ? "System: ON" : "System: OFF");
        // 2번째 줄에 현재 모드 유지
        LCD_UpdateModeStatus();
    }
}


void ProcessCommand(char *cmd) {
    char *trimmed = Trim(cmd);
    if (!trimmed) return;
    // 자율주행 모드에서는 입력값을 그대로 사용(대문자 변환하지 않음)
    char *token = strtok(trimmed, "*");
    if(token) {
        if(strcmp(token, "1") == 0)      currentCommand = CMD_FORWARD;
        else if(strcmp(token, "2") == 0) currentCommand = CMD_RIGHT;
        else if(strcmp(token, "3") == 0) currentCommand = CMD_BACKWARD;
        else if(strcmp(token, "4") == 0) currentCommand = CMD_LEFT;
        else currentCommand = CMD_NONE;
    }
    if(currentCommand != CMD_NONE) {
        commandActive = 1;
        commandStep = 0;
        commandStartTime = HAL_GetTick();
    }
    free(trimmed);
}


// 수동 주행
void ManualDrive(char input) {
    if (mode == 0 && onOff == 1) {
        switch (input) {
            case '1':
                printf("Moving FORWARD\n\r");
                LCD_Print_With_Clear("Moving FORWARD");
                motor_forward();
                currentCommand = CMD_FORWARD;
                break;
            case '2':
                printf("Turning RIGHT\n\r");
                LCD_Print_With_Clear("Turning RIGHT");
                motor_right();
                currentCommand = CMD_RIGHT;
                break;
            case '3':
                printf("Moving BACKWARD\n\r");
                LCD_Print_With_Clear("Moving BACKWARD");
                motor_backward();
                currentCommand = CMD_BACKWARD;
                break;
            case '4':
                printf("Turning LEFT\n\r");
                LCD_Print_With_Clear("Turning LEFT");
                motor_left();
                currentCommand = CMD_LEFT;
                break;
            default:
                printf("Invalid manual command\n\r");
                LCD_Print_With_Clear("Invalid Command");
                motor_stop();
                return;
        }
        commandActive = 1;
        commandStartTime = HAL_GetTick();
    }
}


// UART 수신 완료 콜백 함수 (블루투스 명령 추가)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) // Bluetooth UART
    {
        if (dum1 == 'C' || dum1 == 'c' || dum1 == 'D' || dum1 == 'd') {
            ProcessBluetoothCommand(dum1);
        }
        else if (dum1 == '1' || dum1 == '2' || dum1 == '3' || dum1 == '4') {
            ManualDrive(dum1);
        }
        else
        {
            buf1[t1] = dum1;
            if (dum1 == '0') // 명령 종료
            {
                buf1[t1 + 1] = '\0';
                if (onOff && mode == 1) // 자율주행 모드에서만 명령 처리
                {
                    EnqueueCommand(buf1);
                }
                t1 = 0;
            }
            else
            {
                t1++;
                if (t1 >= BUF_SIZE - 1) t1 = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &dum1, 1);
    }
    else if (huart == &huart2) // 터미널 입력
    {
        buf2[t2] = dum2;
        HAL_UART_Transmit(&huart2, buf2 + t2, 1, 10);

        if (dum2 == '\r')
        {
            HAL_UART_Transmit(&huart2, "\n", 1, 10);
            buf2[t2 + 1] = '\n';
            HAL_UART_Transmit(&huart1, buf2, t2 + 2, 10);
            t2 = 0;
        }
        else
        {
            t2++;
            if (t2 >= BUF_SIZE - 1) t2 = 0;
        }f
        HAL_UART_Receive_IT(&huart2, &dum2, 1);
    }
}

// 상태 머신 함수: 각 명령별로 단계별 실행
void MAINCommand() {
    if (!commandActive) return;

    uint32_t elapsedTime = HAL_GetTick() - commandStartTime;

    switch (currentCommand) {
        case CMD_FORWARD:
            LCD_Print_With_Clear("Moving FORWARD");
            if (elapsedTime > 1000) {
                printf("Stop FORWARD\n\r");
                LCD_Print_With_Clear("Stop");
                motor_stop();
                commandActive = 0;
                currentCommand = CMD_NONE;
            }
            break;
        case CMD_RIGHT:S
            LCD_Print_With_Clear("Turning RIGHT");
            if (elapsedTime > 500) {
                printf("Stop RIGHT TURN\n\r");
                LCD_Print_With_Clear("Stop");
                motor_stop();
                commandActive = 0;
                currentCommand = CMD_NONE;
            }
            break;
        case CMD_BACKWARD:
            LCD_Print_With_Clear("Moving BACKWARD");
            if (elapsedTime > 1000) {
                printf("Stop BACKWARD\n\r");
                LCD_Print_With_Clear("Stop");
                motor_stop();
                commandActive = 0;
                currentCommand = CMD_NONE;
            }
            break;
        case CMD_LEFT:
            LCD_Print_With_Clear("Turning LEFT");
            if (elapsedTime > 500) {
                printf("Stop LEFT TURN\n\r");
                LCD_Print_With_Clear("Stop");
                motor_stop();
                commandActive = 0;
                currentCommand = CMD_NONE;
            }
            break;
        default:
            commandActive = 0;
            currentCommand = CMD_NONE;
            motor_stop();
            break;
    }
}

//ProgramStart("Timer - PWM : uSonic Trigger 10us control");
//HAL_TIM_Base_Start_IT(&htim2);
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PWM out


 int dr = 23, dr1 = 23, dr2 = 25, dr3 = 30;
 int dl = 23, dl1 = 23, dl2 = 25, dl3 = 30;

 void motor_forward() // forward
 {
      HAL_GPIO_WritePin(Dir_A1_GPIO_Port, Dir_A1_Pin, 0);
      HAL_GPIO_WritePin(Dir_A2_GPIO_Port, Dir_A2_Pin, 1);
      HAL_GPIO_WritePin(Dil_A1_GPIO_Port, Dil_A1_Pin, 0);
      HAL_GPIO_WritePin(Dil_A2_GPIO_Port, Dil_A2_Pin, 1);

      dr = 30;
      dl = 30;

      htim3.Instance->CCR3 = htim3.Instance->ARR * dl / 100;
     htim3.Instance->CCR1 = htim3.Instance->ARR * dr / 100;
  }

 void motor_backward() // backward
  {
	 //htim3.Instance->CCR1 = 0;

      HAL_GPIO_WritePin(Dir_A1_GPIO_Port, Dir_A1_Pin, 1);
      HAL_GPIO_WritePin(Dir_A2_GPIO_Port, Dir_A2_Pin, 0);
      HAL_GPIO_WritePin(Dil_A1_GPIO_Port, Dil_A1_Pin, 1);
      HAL_GPIO_WritePin(Dil_A2_GPIO_Port, Dil_A2_Pin, 0);

      dr = 30;
      dl = 30;

      htim3.Instance->CCR3 = htim3.Instance->ARR * dr / 100;
     htim3.Instance->CCR1 = htim3.Instance->ARR * dl / 100;
  }

  void motor_right() // right
  {
     HAL_GPIO_WritePin(Dir_A1_GPIO_Port, Dir_A1_Pin, 0);
     HAL_GPIO_WritePin(Dir_A2_GPIO_Port, Dir_A2_Pin, 1);
     HAL_GPIO_WritePin(Dil_A1_GPIO_Port, Dil_A1_Pin, 1);
     HAL_GPIO_WritePin(Dil_A2_GPIO_Port, Dil_A2_Pin, 0);

     dr = 40;
     dl = 20;

      htim3.Instance->CCR3 = htim3.Instance->ARR * dl / 100;
     htim3.Instance->CCR1 = htim3.Instance->ARR * dr / 100;

   //  HAL_Delay(500);
  }

  void motor_left() // left
  {
     HAL_GPIO_WritePin(Dir_A1_GPIO_Port, Dir_A1_Pin, 1);
     HAL_GPIO_WritePin(Dir_A2_GPIO_Port, Dir_A2_Pin, 0);
     HAL_GPIO_WritePin(Dil_A1_GPIO_Port, Dil_A1_Pin, 0);
     HAL_GPIO_WritePin(Dil_A2_GPIO_Port, Dil_A2_Pin, 1);

     dr = 20;
     dl = 40;

      htim3.Instance->CCR3 = htim3.Instance->ARR * dl / 100;
     htim3.Instance->CCR1 = htim3.Instance->ARR * dr / 100;

   //  HAL_Delay(500);
  }

  void motor_stop() // stop
  {
     dr = 1;
     dl = 1;
      htim3.Instance->CCR3 = htim3.Instance->ARR * dl / 100;
     htim3.Instance->CCR1 = htim3.Instance->ARR * dr / 100;
    //  HAL_Delay(100);
  }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ProgramStart("Code Test");
  i2c_init(&hi2c1); i2c_scan();
  LCD_init();
  printf("Waiting for mode select (C/c)...\n\r");
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // motor PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // motor PWM
  HAL_UART_Receive_IT(&huart1, &dum1, 1); // bluetooth usart
  HAL_UART_Receive_IT(&huart2, &dum2, 1); // putty usart
  Start_Ultrasonic_Measurement(); // ultrasonic echo
  ultrasonic_trigger_setup(); // ultrasonic trigger
//  char command[BUF_SIZE];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char command[BUF_SIZE];
  while (1)
    {


  	   if (onOff) // 시스템이 ON 상태일 때만 동작
        {
            if (mode == 1) { // 자율주행 모드
//                if (!commandActive) {
//                    if (DequeueCommand(command)) {
//                        ProcessCommand(command);
////                    	auto_drive();
//                    }
//                }
//                if (commandActive) {
////              	  MAINCommand();
//                	auto_drive();
//                }
            	//auto_drive();
            	auto_motor();
            }
            else if (mode == 0) { // 수동 주행 모드
                // 수동 모드의 명령은 UART 콜백에서 ManualDrive()로 바로 설정됨.
                if (commandActive) {
              	  MAINCommand();
                }
            }
        }
        else { // 시스템 OFF 상태면 모든 동작 정지
            commandActive = 0;
            currentCommand = CMD_NONE;
        }
  	   // 현재 모드 상태를 지속적으로 LCD에 업데이트
  	       LCD_UpdateModeStatus();
  	       printf("Distance : front: %fcm front_right: %fcm front_left: %fcm back: %fcm\r\n", dist, f_rdist, f_ldist, b_dist);
  	       HAL_Delay(100); // 0.5초마다 LCD 상태 갱신
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 10;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 10;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Dil_A2_Pin|Dil_A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Dir_A1_Pin|Dir_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dil_A2_Pin Dil_A1_Pin */
  GPIO_InitStruct.Pin = Dil_A2_Pin|Dil_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_up_Pin */
  GPIO_InitStruct.Pin = Echo_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_up_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dir_A1_Pin Dir_A2_Pin */
  GPIO_InitStruct.Pin = Dir_A1_Pin|Dir_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
