#include "hx711.h"

// 보정값 저장
float scale_factor = 1.0;  // 변환값 (로드셀 보정값)
long offset = 0;  // 영점(제로포인트) 보정값

// 마이크로초 단위 지연 함수 (DWT 레지스터 이용)
void delayMicroseconds(uint32_t us) {
    uint32_t start = DWT->CYCCNT;  // 현재 CPU 클럭 카운트 저장
    uint32_t ticks = (SystemCoreClock / 1000000) * us;  // 1μs당 클럭 주기 계산
    while ((DWT->CYCCNT - start) < ticks);  // 지정된 시간만큼 대기
}

// HX711 데이터 읽기
long HX711_Read(void) {
    long count = 0;
    uint8_t i;
    uint32_t timeout = HAL_GetTick();

    // DOUT (PB5)이 LOW가 될 때까지 대기 (데이터 준비 확인)
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) {
           if (HAL_GetTick() - timeout > 1000) {  // 1초(1000ms) 이상 대기하면 중단
               printf("HX711 Timeout! Check wiring.\n");
               return -1;  // 오류 코드 반환
           }
       }
    // 24비트 데이터 읽기
    for (i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // SCK HIGH
        delayMicroseconds(1);
        count = count << 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  // SCK LOW
        delayMicroseconds(1);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
            count++;
    }

    // 추가 SCK 펄스 (게인 설정)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    delayMicroseconds(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    return count ^ 0x800000;  // HX711의 2의 보수 변환
}

// HX711 영점 보정 (tare)
void HX711_Tare(uint8_t times) {
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += HX711_Read();
        HAL_Delay(10);
    }
    offset = sum / times;
}

// HX711 무게 측정
float HX711_GetWeight(uint8_t times) {
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += HX711_Read();
        HAL_Delay(10);
    }
    return ((sum / times) - offset) / scale_factor;
}

// HX711 보정값 설정 (스케일 팩터)
void HX711_SetScale(float scale) {
    scale_factor = scale;
}

// HX711 영점 보정값 설정
void HX711_SetOffset(long new_offset) {
    offset = new_offset;
}
