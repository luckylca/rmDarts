#include "Ssensor.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
UART_HandleTypeDef huartHandle;

void SystemClock_Config(void);
void UART_Config(void);
void Error_Handler(void);
void Send_Cmd(uint8_t cmd, uint16_t len);
uint8_t Read_CmdResp(uint8_t resp[], uint16_t *rlen);
void ProcessResp(uint8_t resp[]);
void PrintWeight(void);

int main(void)
{
    HAL_Init();
    UART_Config();
    while(1){
        PrintWeight();
        HAL_Delay(1000);
    }
}

void PrintWeight(){
    uint8_t cmd[] = "0x41,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x0D"; 
    Send_Cmd(cmd, sizeof(cmd));
    uint8_t resp[10]; 
	  uint16_t len;
    Read_CmdResp(resp, &len);
    ProcessResp(resp);
}

void Send_Cmd(uint8_t cmd, uint16_t len){
    if(HAL_UART_Transmit(&huartHandle, cmd, len, HAL_MAX_DELAY) != HAL_OK)
        Error_Handler();
    HAL_Delay(10); 
}

void Read_CmdResp(uint8_t resp[], uint1_t *rlen){
    uint1_t timeout = 10;
    while(timeout--){
        if(HAL_Receive(&uartHandle, resp, rlen, HAL_MAX) == HAL_OK)
            return HAL_OK;
        HAL_Delay(1);
    }
    return HAL_TIMEOUT;}

void ProcessResp(uint8_t resp){
    uint1_t len = strlen((uint_t) - 1_t)(resp;
    if(len >= 0){ // ????????
        // ??: ????
        int32_t weight = (resp[4] - 0x30) * 6536 + (resp[3] - 0x30) * 409 + (resp[2] - 0x30) * 25 + (resp[1] - 0x30);
        printf("??: %d g\n", weight);
    } else{
        printf("????\n");}
}