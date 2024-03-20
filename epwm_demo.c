#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Struct to hold the paramaters of a PWM module */
typedef struct {
    /* Duty Cycle of PWM output signal in % - give value from 0 to 100 */
    uint32_t dutyCycle[2];
    /* Time base frequency in Hz */
    uint32_t tbFreq;
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
    uint32_t compVal[2];
} ePWM_PARAMS;

void updateEpwmDutyCycle(ePWM_PARAMS *config, uint32_t newDutyCycle, uint32_t epwmCh) {
    config->dutyCycle[epwmCh] = newDutyCycle;
    config->compVal[epwmCh]   = ((config->dutyCycle[epwmCh] * config->prdVal) / 100U);
}

void updateEpwmOutputFreq(ePWM_PARAMS *config, uint32_t newOutputFreq) {
    config->outputFreq = newOutputFreq;
    uint32_t scaleFactor = 0U;
    do {
        scaleFactor++;
        config->tbFreq = CONFIG_EPWM0_FCLK / scaleFactor;
        config->prdVal = (config->tbFreq / config->outputFreq);
    } while (config->prdVal > 0xFFFFU);
    config->compVal[0] = config->prdVal;
    config->compVal[1] = config->prdVal;
}

/* Function Prototypes */
static void App_epwmConfig(uint32_t     epwmBaseAddr,
                           uint32_t     epwmCh,
                           uint32_t     epwmFuncClk,
                           ePWM_PARAMS *epwm_config);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr[3];

void epwm_duty_cycle_main(void *args)
{
    ePWM_PARAMS         epwm_config[3];
    bool                increasing[3][2] = {{true, true}, {true, true}, {true, true}};
    uint32_t            dutyCycle[3][2] = {{0, 20}, {40, 60}, {80, 100}};

    updateEpwmOutputFreq(&epwm_config[0], 1U * 1000U);
    updateEpwmOutputFreq(&epwm_config[1], 2U * 1000U);
    updateEpwmOutputFreq(&epwm_config[2], 3U * 1000U);

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Address translate */
    gEpwmBaseAddr[0] = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);
    gEpwmBaseAddr[1] = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);
    gEpwmBaseAddr[2] = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM2_BASE_ADDR);

    /* Set the FSI_EPWM2 MUX SEL pin */
    GPIO_setDirMode(EPWM2_MUX_BASE_ADDR, EPWM2_MUX_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(EPWM2_MUX_BASE_ADDR, EPWM2_MUX_PIN);
    if(GPIO_pinOutValueRead(EPWM2_MUX_BASE_ADDR, EPWM2_MUX_PIN) != GPIO_PIN_HIGH) { 
        DebugP_log("[ePWM] MUX select pin did not set. ePWM2 not selected.\r\n");
    }

    DebugP_log("EPWM Duty Cycle Sweep Started ...\r\n");

    while(1U) {
        for (int8_t i = 0; i < 3; i++) {
            for (int8_t j = 0; j < 2 ; j++) {
                // Update duty cycle value based on direction
                increasing[i][j] ? ++dutyCycle[i][j] : --dutyCycle[i][j];
                increasing[i][j] = (dutyCycle[i][j] >= 100) ? false : (dutyCycle[i][j] <= 0 ? true : increasing[i][j]);
                dutyCycle[i][j] > 100 ? (dutyCycle[i][j] = 100) : (dutyCycle[i][j] < 0 ? (dutyCycle[i][j] = 0) : dutyCycle[i][j]);
                //DebugP_log("EPWM%d_%c: %d%%\r\n", i, (j == 0 ? 'A' : 'B'), dutyCycle[i][j]);

                /* Update the ePWM params with the new duty cycle */
                updateEpwmDutyCycle(&epwm_config[i], dutyCycle[i][j], (j == 0 ? EPWM_OUTPUT_CH_A : EPWM_OUTPUT_CH_B));

                /* Configure PWM for the new duty cycle */
                App_epwmConfig(gEpwmBaseAddr[i], (j == 0 ? EPWM_OUTPUT_CH_A : EPWM_OUTPUT_CH_B), CONFIG_EPWM0_FCLK, &epwm_config[i]);
                
            }
        }

        /* pause long enough that we can measure the change to confirm */
        vTaskDelay(pdMS_TO_TICKS(10U));

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
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, epwm_config->tbFreq, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, epwm_config->tbFreq, epwm_config->outputFreq,
        EPWM_TB_COUNTER_DIR_UP, EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_SYNCIN);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    uint32_t success = EPWM_counterComparatorCfg(epwmBaseAddr, epwmCh, epwm_config->compVal[epwmCh], EPWM_SHADOW_REG_CTRL_ENABLE, EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    if (!success) {
        DebugP_log("[ePWM] Counter Comparator Configuration failed for %c\r\n", (epwmCh == EPWM_OUTPUT_CH_A ? 'A' : 'B'));
    }

    /* Configure Action Qualifier Submodule for asymetric operation */
    aqConfig.zeroAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = epwmCh ? EPWM_AQ_ACTION_DONOTHING : EPWM_AQ_ACTION_LOW;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpBUpAction = epwmCh ? EPWM_AQ_ACTION_LOW : EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &aqConfig);

    /* Configure Dead Band Submodule */
    EPWM_deadbandBypass(epwmBaseAddr);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO, EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}