#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Output channel - A or B */
#define APP_EPWM_OUTPUT_CH              (EPWM_OUTPUT_CH_A)
/* TB frequency in Hz - so that /4 divider is used */
#define APP_EPWM_TB_FREQ                (CONFIG_EPWM0_FCLK / 4U)

/* Struct to hold the paramaters of a PWM module */
typedef struct {
    /* Duty Cycle of PWM output signal in % - give value from 0 to 100 */
    uint32_t dutyCycle;
    /* Frequency of PWM output signal in Hz - 1 KHz is selected */
    uint32_t outputFreq;
    /*
    *  PRD value - this determines the period
    *  PRD = (TBCLK/PWM FREQ) / 2
    *  /2 is added becasue up&down counter is selected. So period is 2 times
    */
    uint32_t prdVal;
    /*
    *  COMPA value - this determines the duty cycle
    *  COMPA = (PRD - ((dutycycle * PRD) / 100)
    */
    uint32_t compVal;
} ePWM_PARAMS;

void updateEpwmDutyCycle(ePWM_PARAMS *config, uint32_t newDutyCycle) {
    config->dutyCycle = newDutyCycle;
    config->compVal = (config->prdVal - ((config->dutyCycle * config->prdVal) / 100U));
    config->prdVal = ((APP_EPWM_TB_FREQ / config->outputFreq) / 2U);
    config->compVal = (config->prdVal - ((config->dutyCycle * config->prdVal) / 100U));
}

void updateEpwmOutputFreq(ePWM_PARAMS *config, uint32_t newOutputFreq) {
    config->outputFreq = newOutputFreq;
    config->prdVal = ((APP_EPWM_TB_FREQ / config->outputFreq) / 2U);
    config->compVal = (config->prdVal - ((config->dutyCycle * config->prdVal) / 100U));
    config->prdVal = ((APP_EPWM_TB_FREQ / config->outputFreq) / 2U);
    config->compVal = (config->prdVal - ((config->dutyCycle * config->prdVal) / 100U));
}

/* Function Prototypes */
static void App_epwmConfig(uint32_t     epwmBaseAddr,
                           uint32_t     epwmCh,
                           uint32_t     epwmFuncClk,
                           ePWM_PARAMS *epwm_config);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr;

void epwm_duty_cycle_main(void *args)
{
    bool                increasing = true;
    uint32_t            dutyCycle  = 0;

    ePWM_PARAMS epwm_config = {
        .dutyCycle  = 0U,
        .outputFreq = 1U * 1000U,
    };

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Address translate */
    gEpwmBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    DebugP_log("EPWM Duty Cycle Sweep Started ...\r\n");

    while(1U) {
        // Update duty cycle value based on direction
        increasing ? ++dutyCycle : --dutyCycle;
        increasing = (dutyCycle >= 100) ? false : (dutyCycle <= 0 ? true : increasing);

        /* Update the ePWM params with the new duty cycle */
        updateEpwmDutyCycle(&epwm_config, dutyCycle);

        /* Configure PWM for the new duty cycle */
        App_epwmConfig(gEpwmBaseAddr, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK, &epwm_config);

        /* The sweep should happen over 1 second */
        vTaskDelay(pdMS_TO_TICKS(1000U / 100U));
    }

    Board_driversClose();
    Drivers_close();
}

static void App_epwmConfig(uint32_t     epwmBaseAddr,
                           uint32_t     epwmCh,
                           uint32_t     epwmFuncClk,
                           ePWM_PARAMS *epwm_config)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, epwm_config->outputFreq,
        EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_SYNCIN);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        epwm_config->compVal, EPWM_SHADOW_REG_CTRL_ENABLE,
        EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
        epwm_config->compVal, EPWM_SHADOW_REG_CTRL_ENABLE,
        EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    aqConfig.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_LOW;
    aqConfig.cmpBUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpBDownAction = EPWM_AQ_ACTION_LOW;
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &aqConfig);

    /* Configure Dead Band Submodule */
    EPWM_deadbandBypass(epwmBaseAddr);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}
