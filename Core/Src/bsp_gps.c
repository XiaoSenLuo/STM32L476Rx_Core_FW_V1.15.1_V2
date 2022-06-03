//
// Created by XIAOSENLUO on 2022/5/25.
//

#include "bsp_gps.h"


static GPIO_TypeDef *gpsPPSPort = NULL;
int32_t gpsPPSPin = -1;
int gpsPPSIRQn = -255;

void gps_pps_isr_install(GPIO_TypeDef * ppsPort, int32_t ppsPin, isr_function_handle_t fn, void *ctx){

    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_RISING | GPIO_MODE_INPUT,
            .Pin = PIN_MASK(ppsPin),
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };
    if((ppsPort == NULL) || (ppsPin < 0)) return;
    HAL_GPIO_Init(ppsPort, &GPIO_InitStructure);

    gpsPPSPort = ppsPort;
    gpsPPSPin = ppsPin;
    gpsPPSIRQn = gpio_get_irqn(gpsPPSPin);

    ll_gpio_exti_isr_install(gpsPPSPin, fn, ctx);

    HAL_NVIC_SetPriority(gpsPPSIRQn, 1, 0);
    HAL_NVIC_EnableIRQ(gpsPPSIRQn);
}

void gps_pps_isr_uninstall(void){

    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = PIN_MASK(gpsPPSPin),
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };

    if((gpsPPSPort == NULL) || (gpsPPSPin < 0)) return;
    HAL_GPIO_Init(gpsPPSPort, &GPIO_InitStructure);

    HAL_NVIC_DisableIRQ(gpsPPSIRQn);
    ll_gpio_exti_isr_uninstall(gpsPPSPin);
}

void gps_pps_irq_disable(void){
    if((gpsPPSPort == NULL) || (gpsPPSPin < 0)) return;
    HAL_NVIC_SetPriority(gpsPPSIRQn, 0, 0);
    HAL_NVIC_EnableIRQ(gpsPPSIRQn);
}


void gps_pps_irq_enable(void){
    if((gpsPPSPort == NULL) || (gpsPPSPin < 0)) return;
    HAL_NVIC_DisableIRQ(gpsPPSIRQn);
}

#include "usart.h"
#include "string.h"
#include "stdarg.h"

static void config_string(char* ptr, size_t cnt, const char* format, ...){
    uint8_t cs, n;
    va_list arp;

    va_start(arp, format);
    memset(ptr, 0, cnt);
    n = vsnprintf(ptr, cnt, format, arp);
    va_end(arp);
    cs = minmea_checksum(ptr);
    snprintf(&ptr[n], (cnt - n), "%X\r\n", cs);
}


void gps_config_output(const gps_ctrl_output_t *out){
    if(out == NULL) return;
    char pcas[64] = {'\0'}, cs = 0, n = 0;

    config_string(pcas, sizeof(pcas), "$PCAS03,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,%d,%d,,,,%d*",
                  out->nGGA, out->nGLL, out->nGSA,
                  out->nGSV, out->nRMC, out->nVTG,
                  out->nZDA, out->nANT, out->nDHV,
                  out->nLPS, out->nUTC, out->nGST,
                  out->nTIM
    ); // 设置输出语句, 顺序不可改变

    lpuart1_write_bytes(NULL, pcas, strlen(pcas), 10000);
}
