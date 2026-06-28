/*
*********************************************************************************************************
* app.c - Dual Conveyor RTOS Factory Safety Simulation
* + HC-SR04 Ultrasonic Person Detection
* + SG90 Servo Motor Door Control
* + PA4 Potentiometer Maintenance Level Selector
* + Profit / Cost / Net Profit Factory Economy
*
* Target : NUCLEO-F429ZI
* RTOS   : uC/OS-III
* Sensor : ADXL345 on I2C1 (PB8=SCL, PB9=SDA)
* HC-SR04 Ultrasonic   TRIG=PA2, ECHO=PA3
* Output : SG90 Servo PWM on PA0 (TIM5 CH1)
* Input  : 10k Potentiometer middle pin on PA4 (ADC1 CH4)
*
* SUMMARY (v15 - Unified Emergency ForceServo90Deg Architecture):
*
* Person Detection States (ultrasonic):
* >= 45 cm : No person   -> normal conveyor behaviour
* 9-44 cm  : WARNING     -> warning beep + magenta light + door partially closing
* <= 8 cm  : EMERGENCY   -> all off, door locked 90 deg via unified routine
*
* Servo Door Positions:
* NORMAL   : open          (~1.00 ms pulse = 0 deg)
* WARNING  : partial close (~1.25 ms pulse = ~45 deg)
* DANGER/STOP/EMERGENCY : closed/locked (~1.75 ms pulse = ~90 deg)
*
*********************************************************************************************************
*/

#include <includes.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* =========================================================
 * Configuration
 * =========================================================*/

#define APP_TASK_STK_SIZE             512u
#define CMD_BUF_SIZE                   64u
#define I2C_TIMEOUT                 50000u

/* PA4 potentiometer -> ADC1 Channel 4 */
#define POT_GPIO                       GPIOA
#define POT_PIN                        GPIO_Pin_4
#define POT_ADC                        ADC1
#define POT_ADC_CHANNEL                ADC_Channel_4
#define POT_ADC_MAX                    4095u
#define POT_SAMPLE_PERIOD_MS           200u

/* Factory economy / maintenance model */
#define MACHINE_HEALTH_MAX             100u
#define MACHINE_HEALTH_MIN               0u
#define MAINT_ALLOWED_HEALTH            70u
#define MAINT_LEVEL_COUNT                5u
#define PROFIT_PER_PRODUCT              10u

#define CONVEYOR_A_MASK  (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)
#define CONVEYOR_B_MASK  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define CONVEYOR_ALL_MASK (CONVEYOR_A_MASK | CONVEYOR_B_MASK)

/* RGB: PC4=RED, PC3=GREEN, PC2=BLUE */
#define RGB_RED_PIN                   GPIO_Pin_4
#define RGB_GREEN_PIN                 GPIO_Pin_3
#define RGB_BLUE_PIN                  GPIO_Pin_2
#define RGB_ALL_MASK       (RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN)
#define BUZZER_PIN                    GPIO_Pin_5

#define START_BUTTON_PIN              GPIO_Pin_13
#define MAINT_BUTTON_PIN              GPIO_Pin_0
#define ESTOP_BUTTON_PIN              GPIO_Pin_1

/* HC-SR04: PA2=TRIG, PA3=ECHO */
#define ULTRA_TRIG_PIN                GPIO_Pin_2
#define ULTRA_ECHO_PIN                GPIO_Pin_3
#define ULTRA_GPIO                    GPIOA

/*
 * SG90 Servo on PA0 via TIM5 CH1.
 * 1000 us = 0 deg  (open)
 * 1250 us = ~45 deg (warning / half)
 * 1750 us = ~90 deg (closed / locked / STOP)
 */
#define SERVO_PIN                     GPIO_Pin_0     /* PA0 */
#define SERVO_PWM_PERIOD_US           20000u
#define SERVO_PULSE_OPEN_US           1000u          /* 0 deg   - door open    */
#define SERVO_PULSE_WARN_US           1250u          /* ~45 deg - door half    */
#define SERVO_PULSE_CLOSED_US         1750u          /* ~90 deg - door closed / locked / STOP */

/* How many 20-ms PWM cycles to hold the pulse after a hard move to ensure physical transit */
#define SERVO_CLOSE_HOLD_CYCLES        30u           /* hold 90 deg for 600 ms */
#define SERVO_OPEN_HOLD_CYCLES         30u           /* hold  0 deg for 600 ms */

/* Ultrasonic distance thresholds (cm). */
#define ULTRA_NEAR_CM                    8u
#define ULTRA_WARN_CM                   45u
#define ULTRA_MAX_RANGE_CM             400u
#define ULTRA_VALID_MIN_CM               1u

#define ULTRA_ENTER_NEAR_COUNT           2u
#define ULTRA_ENTER_WARN_COUNT           2u
#define ULTRA_LEAVE_WARN_COUNT           4u
#define ULTRA_MEASURE_PERIOD_MS         60u

/* Servo animation during WARNING */
#define SERVO_STEPS                     20u
#define SERVO_STEP_DELAY_MS             50u

#define BUZZER_IS_PASSIVE                1u

/* ADXL345 */
#define ADXL345_ADDR                    0xA6u
#define ADXL_REG_DEVID                  0x00u
#define ADXL_REG_POWER_CTL              0x2Du
#define ADXL_REG_DATA_FORMAT            0x31u
#define ADXL_REG_DATAX0                 0x32u
#define ADXL_EXPECTED_DEVID             0xE5u

/* Vibration thresholds */
#define VIB_WARN_UP                       35u
#define VIB_WARN_DOWN                     20u
#define VIB_DANGER_UP                    120u
#define VIB_DANGER_DOWN                   70u
#define VIB_EMERGENCY_UP                 380u

#define SENSOR_PERIOD_MS                  50u
#define SENSOR_LOG_PERIOD_SAMPLES        200u

#define WARN_CONFIRM_SAMPLES               2u
#define DANGER_CONFIRM_SAMPLES             3u
#define EMERGENCY_CONFIRM_SAMPLES          6u
#define SAFE_CONFIRM_SAMPLES               8u
#define WARNING_MIN_SAMPLES               20u
#define RECOVERY_MIN_SAMPLES              30u
#define DANGER_MIN_SAMPLES                20u

#define DELAY_NORMAL_MS                  350u
#define DELAY_WARNING_MS                 250u
#define DELAY_DANGER_MS                  120u
#define DELAY_RECOVERY_MS                500u

/* Task priorities */
#define PRIO_EMERGENCY                     2u
#define PRIO_BUTTON                        4u
#define PRIO_SENSOR                        5u
#define PRIO_CONTROL                       6u
#define PRIO_RGB                           7u
#define PRIO_CONVEYOR_A                    8u
#define PRIO_CONVEYOR_B                    9u
#define PRIO_BUZZER                       10u
#define PRIO_ULTRASONIC                   11u
#define PRIO_MAINTENANCE                  12u
#define PRIO_SERVO                         3u
#define PRIO_USART                        14u

/* =========================================================
 * Types
 * =========================================================*/

typedef enum {
    SYS_STATE_NORMAL = 0,
    SYS_STATE_WARNING,
    SYS_STATE_RECOVERY,
    SYS_STATE_DANGER,
    SYS_STATE_EMERGENCY
} SYSTEM_STATE;

typedef enum {
    DOOR_OPEN = 0,
    DOOR_CLOSING,
    DOOR_HALF,
    DOOR_CLOSED,
    DOOR_LOCKED
} DOOR_STATE;

typedef struct {
    int16_t  X;
    int16_t  Y;
    int16_t  Z;
    uint32_t VibA;
    uint32_t VibB;
    CPU_BOOLEAN Valid;
} SENSOR_DATA;

typedef struct {
    SYSTEM_STATE State;
    uint32_t HighCount;
    uint32_t SafeCount;
    uint32_t HoldCount;
    uint32_t RecoveryCount;
} STATE_TRACKER;

typedef enum {
    PERSON_NONE = 0,
    PERSON_WARN,
    PERSON_NEAR
} PERSON_STATE;

typedef struct {
    uint8_t      Level;
    const char  *Name;
    uint32_t     Cost;
    uint32_t     HealthGain;
    uint32_t     DowntimeSec;
} MAINTENANCE_LEVEL_INFO;

static const MAINTENANCE_LEVEL_INFO MaintenanceTable[MAINT_LEVEL_COUNT] = {
    {1u, "Level 1: Inspection",          50u, 10u,  3u},
    {2u, "Level 2: Minor Repair",       120u, 20u,  5u},
    {3u, "Level 3: Standard Service",   220u, 35u,  7u},
    {4u, "Level 4: Major Repair",       350u, 55u, 10u},
    {5u, "Level 5: Full Overhaul",      500u, 80u, 15u}
};

/* =========================================================
 * Task objects and stacks
 * =========================================================*/

static OS_TCB  AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB  EmergencyTaskTCB;
static CPU_STK EmergencyTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  ButtonTaskTCB;
static CPU_STK ButtonTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  SensorTaskTCB;
static CPU_STK SensorTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  ControlTaskTCB;
static CPU_STK ControlTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  ConveyorATaskTCB;
static CPU_STK ConveyorATaskStk[APP_TASK_STK_SIZE];
static OS_TCB  ConveyorBTaskTCB;
static CPU_STK ConveyorBTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  RgbTaskTCB;
static CPU_STK RgbTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  BuzzerTaskTCB;
static CPU_STK BuzzerTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  UltrasonicTaskTCB;
static CPU_STK UltrasonicTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  MaintenanceTaskTCB;
static CPU_STK MaintenanceTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  ServoTaskTCB;
static CPU_STK ServoTaskStk[APP_TASK_STK_SIZE];
static OS_TCB  UsartTaskTCB;
static CPU_STK UsartTaskStk[APP_TASK_STK_SIZE];

/* RTOS objects */
static OS_SEM   EmergencySem;
static OS_SEM   SensorDataSem;
static OS_SEM   RgbUpdateSem;
static OS_SEM   ServoUpdateSem;
static OS_MUTEX UartMutex;

/* =========================================================
 * Shared application state
 * =========================================================*/

static volatile CPU_BOOLEAN   EmergencyActive        = DEF_FALSE;
static volatile CPU_BOOLEAN   SensorEmergencyRequest = DEF_FALSE;
static volatile CPU_BOOLEAN   SystemRun              = DEF_FALSE;

static volatile SYSTEM_STATE  ConveyorAState  = SYS_STATE_NORMAL;
static volatile SYSTEM_STATE  ConveyorBState  = SYS_STATE_NORMAL;
static volatile SYSTEM_STATE  OverallState    = SYS_STATE_NORMAL;

static volatile uint32_t      ProductCountA;
static volatile uint32_t      ProductCountB;

static volatile uint32_t      FactoryRunTimeSec       = 0u;
static volatile uint32_t      MachineHealth           = MACHINE_HEALTH_MAX;
static volatile uint32_t      TotalMaintenanceCost    = 0u;
static volatile uint32_t      FactoryProfit           = 0u;
static volatile int32_t       FactoryNetProfit        = 0;
static volatile uint32_t      PotAdcRaw               = 0u;
static volatile uint8_t       SelectedMaintLevel      = 1u;
static volatile CPU_BOOLEAN   MaintenanceInProgress  = DEF_FALSE;
static volatile CPU_BOOLEAN   DoorHoldClosedUntilStart = DEF_FALSE;

static volatile CPU_BOOLEAN   MaintButtonIdleSaved    = DEF_FALSE;
static volatile uint8_t       MaintButtonIdlePulldown = Bit_RESET;
static volatile uint8_t       MaintButtonIdlePullup   = Bit_SET;

static volatile uint32_t      ConveyorADelayMs = DELAY_NORMAL_MS;
static volatile uint32_t      ConveyorBDelayMs = DELAY_NORMAL_MS;

static volatile PERSON_STATE  CurrentPersonState     = PERSON_NONE;
static volatile PERSON_STATE  PreviousPersonState    = PERSON_NONE;
static volatile uint32_t      LastDistanceCm         = ULTRA_MAX_RANGE_CM;

static volatile DOOR_STATE    CurrentDoorState       = DOOR_OPEN;

static volatile CPU_BOOLEAN   SensorLogEnabled       = DEF_FALSE;

/*
 * ServoHardLock:
 * 1 = servo MUST stay at 90 deg; no manual/automatic movement allowed.
 * 0 = servo may move freely.
 */
static volatile uint32_t      ServoHardLock          = 0u;

static uint32_t SysClockHz = 0u;
static SENSOR_DATA LatestSensorData;

/* =========================================================
 * Prototypes
 * =========================================================*/

static void AppTaskStart       (void *p_arg);
static void AppTaskEmergency   (void *p_arg);
static void AppTaskButton      (void *p_arg);
static void AppTaskSensor      (void *p_arg);
static void AppTaskControl     (void *p_arg);
static void AppTaskConveyorA   (void *p_arg);
static void AppTaskConveyorB   (void *p_arg);
static void AppTaskRgb         (void *p_arg);
static void AppTaskBuzzer      (void *p_arg);
static void AppTaskUltrasonic  (void *p_arg);
static void AppTaskMaintenance (void *p_arg);
static void AppTaskServo       (void *p_arg);
static void AppTaskUsart       (void *p_arg);

static void AppObjCreate(void);
static void AppTaskCreate(void);

static void SetupGpio(void);
static void SetupUsart3(void);
static void SetupI2C1(void);
static void SetupEmergencyExti(void);
static void SetupServoTim5(void);
static void SetupAdc1Potentiometer(void);
static void AppEmergencyISR(void);

static void Timer_Init(void);
static void DelayUs(uint32_t us);

static CPU_BOOLEAN I2CWaitEvent     (I2C_TypeDef *i2c, uint32_t event);
static CPU_BOOLEAN I2CWaitWhileFlag (I2C_TypeDef *i2c, uint32_t flag, FlagStatus value);
static void        I2CAbort         (void);
static CPU_BOOLEAN ADXLReadReg      (uint8_t reg, uint8_t *value);
static CPU_BOOLEAN ADXLWriteReg     (uint8_t reg, uint8_t value);
static CPU_BOOLEAN ADXLReadXYZ      (int16_t *x, int16_t *y, int16_t *z);

static uint32_t UltraMeasureCm(void);

static uint32_t PotReadRaw(void);
static uint8_t  PotRawToMaintenanceLevel(uint32_t raw);
static const MAINTENANCE_LEVEL_INFO *GetMaintenanceInfo(uint8_t level);
static void     UpdateFactoryEconomy(void);
static void     ApplySelectedMaintenance(void);
static void     PrintEconomy(void);
static uint8_t  MaintButtonReadWithPull(GPIOPuPd_TypeDef pull);
static CPU_BOOLEAN MaintButtonIsPressed(void);
static void     PrintMaintButtonDebug(void);

static void ServoSetPulseUs    (uint32_t pulse_us);
static void ServoClose90Deg_hw (void);
static void ServoOpen0Deg_hw   (void);
static void ServoWaitFor90Deg  (void);    /* hold 90 deg for physical travel  */
static void ServoWaitFor0Deg   (void);    /* hold  0 deg for physical travel  */
static void ForceServo90Deg    (void);    /* NEW: Unified Emergency Safety Function */
static void ServoMoveSmooth    (uint32_t from_us, uint32_t to_us,
                                 uint32_t steps,  uint32_t step_delay_ms);

static SYSTEM_STATE UpdateVibrationState(STATE_TRACKER *tracker, uint32_t vibration);
static SYSTEM_STATE GetOverallState(void);
static uint32_t     StateToDelay   (SYSTEM_STATE state);
static const char  *StateToString   (SYSTEM_STATE state);
static const char  *PersonStateToString(PERSON_STATE ps);
static const char  *DoorStateToString  (DOOR_STATE   ds);

static void        AllConveyorsOff(void);
static void        StopSystemAndCloseDoor(void);
static CPU_BOOLEAN ConveyorResponsiveDelay(CPU_BOOLEAN conveyor_a,
                                            SYSTEM_STATE expected_state,
                                            uint32_t delay_ms);

static void SetRgb     (SYSTEM_STATE state);
static void BuzzerOff  (void);
static void BuzzerTone   (uint32_t duration_ms, SYSTEM_STATE expected_state);
static void BuzzerSilence(uint32_t duration_ms, SYSTEM_STATE expected_state);

static void UsartPutCharRaw(char c);
static void UsartPrint     (const char *text);
static void PrintStatus    (void);
static void HandleCommand  (char *command);

/* =========================================================
 * main
 * =========================================================*/

int main(void)
{
    OS_ERR err;

    RCC_DeInit();
    BSP_IntDisAll();
    CPU_Init();
    Mem_Init();
    Math_Init();

    OSInit(&err);

    OSTaskCreate(&AppTaskStartTCB,
                 "App Task Start",
                 AppTaskStart,
                 (void *)0u,
                 APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                 APP_CFG_TASK_START_STK_SIZE / 10u,
                 APP_CFG_TASK_START_STK_SIZE,
                 0u, 0u, (void *)0u,
                 OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
                 &err);

    OSStart(&err);
    while (DEF_TRUE) {}
}

/* =========================================================
 * Startup
 * =========================================================*/

static void AppTaskStart(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    BSP_Init();
    BSP_Tick_Init();

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);
#endif

    Timer_Init();

    SetupGpio();
    SetupUsart3();
    SetupI2C1();
    SetupServoTim5();
    SetupAdc1Potentiometer();

    AppObjCreate();
    SetupEmergencyExti();

    UsartPrint("\r\n==================================================\r\n");
    UsartPrint(" Dual Conveyor RTOS Factory Safety Simulation v15\r\n");
    UsartPrint(" + Unified ForceServo90Deg Structural Bottleneck\r\n");
    UsartPrint(" + HC-SR04 Person Detection (PA2=TRIG, PA3=ECHO)\r\n");
    UsartPrint(" + SG90 Servo Door (PA0 / TIM5 CH1)\r\n");
    UsartPrint(" + Potentiometer Maintenance Selector (PA4 / ADC1 CH4)\r\n");
    UsartPrint(" SERVO: OPEN=0deg  HALF=45deg  CLOSED/STOP=90deg\r\n");
    UsartPrint(" STOP CONDITIONS -> servo physically reaches 90 deg FIRST:\r\n");
    UsartPrint("   1) Emergency button (PC1)\r\n");
    UsartPrint("   2) Object detected <= 8 cm (HC-SR04)\r\n");
    UsartPrint("   3) 'stop' command or pause button\r\n");
    UsartPrint("   4) MachineHealth = 0\r\n");
    UsartPrint(" Restart: servo physically opens to 0 deg BEFORE conveyors start.\r\n");
    UsartPrint(" Person: >=45cm=NONE  9-44cm=WARN  <=8cm=EMERGENCY\r\n");
    UsartPrint(" Commands: start stop status count ultra pot maint money clear help\r\n");
    UsartPrint("==================================================\r\n> ");

    AppTaskCreate();
    OSTaskDel((OS_TCB *)0u, &err);
}

static void AppObjCreate(void)
{
    OS_ERR err;

    OSSemCreate(&EmergencySem,   "Emergency Semaphore",    0u, &err);
    OSSemCreate(&SensorDataSem, "Sensor Data Semaphore",  0u, &err);
    OSSemCreate(&RgbUpdateSem,  "RGB Update Semaphore",   0u, &err);
    OSSemCreate(&ServoUpdateSem,"Servo Update Semaphore", 0u, &err);
    OSMutexCreate(&UartMutex,   "USART Mutex",                &err);

    memset(&LatestSensorData, 0, sizeof(LatestSensorData));
}

static void AppTaskCreate(void)
{
    OS_ERR err;

#define CREATE(tcb,stk,fn,prio) \
    OSTaskCreate(&(tcb), #fn, (fn), 0u, (prio), &(stk)[0u], \
                 APP_TASK_STK_SIZE/10u, APP_TASK_STK_SIZE, \
                 0u, 0u, 0u, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err)

    CREATE(EmergencyTaskTCB,  EmergencyTaskStk,  AppTaskEmergency,  PRIO_EMERGENCY);
    CREATE(ButtonTaskTCB,     ButtonTaskStk,     AppTaskButton,     PRIO_BUTTON);
    CREATE(SensorTaskTCB,     SensorTaskStk,     AppTaskSensor,     PRIO_SENSOR);
    CREATE(ControlTaskTCB,    ControlTaskStk,    AppTaskControl,    PRIO_CONTROL);
    CREATE(ConveyorATaskTCB,  ConveyorATaskStk,  AppTaskConveyorA,  PRIO_CONVEYOR_A);
    CREATE(ConveyorBTaskTCB,  ConveyorBTaskStk,  AppTaskConveyorB,  PRIO_CONVEYOR_B);
    CREATE(RgbTaskTCB,        RgbTaskStk,        AppTaskRgb,        PRIO_RGB);
    CREATE(BuzzerTaskTCB,     BuzzerTaskStk,     AppTaskBuzzer,     PRIO_BUZZER);
    CREATE(UltrasonicTaskTCB, UltrasonicTaskStk, AppTaskUltrasonic, PRIO_ULTRASONIC);
    CREATE(MaintenanceTaskTCB,MaintenanceTaskStk,AppTaskMaintenance,PRIO_MAINTENANCE);
    CREATE(ServoTaskTCB,      ServoTaskStk,      AppTaskServo,      PRIO_SERVO);
    CREATE(UsartTaskTCB,      UsartTaskStk,      AppTaskUsart,      PRIO_USART);

#undef CREATE
}

/* =========================================================
 * Servo helpers (v15 - Unified Design)
 * =========================================================*/

static void ServoSetPulseUs(uint32_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_OPEN_US)   { pulse_us = SERVO_PULSE_OPEN_US;   }
    if (pulse_us > SERVO_PULSE_CLOSED_US) { pulse_us = SERVO_PULSE_CLOSED_US; }
    TIM_SetCompare1(TIM5, pulse_us);
}

static void ServoClose90Deg_hw(void)
{
    TIM_SetCompare1(TIM5, SERVO_PULSE_CLOSED_US);
}

static void ServoOpen0Deg_hw(void)
{
    TIM_SetCompare1(TIM5, SERVO_PULSE_OPEN_US);
}

static void ServoWaitFor90Deg(void)
{
    OS_ERR   err;
    uint32_t i;

    for (i = 0u; i < SERVO_CLOSE_HOLD_CYCLES; i++) {
        TIM_SetCompare1(TIM5, SERVO_PULSE_CLOSED_US);
        OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    CurrentDoorState = DOOR_LOCKED;
}

static void ServoWaitFor0Deg(void)
{
    OS_ERR   err;
    uint32_t i;

    for (i = 0u; i < SERVO_OPEN_HOLD_CYCLES; i++) {
        if (ServoHardLock != 0u) {
            ServoClose90Deg_hw();
            return;
        }
        TIM_SetCompare1(TIM5, SERVO_PULSE_OPEN_US);
        OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);
    }

    if (ServoHardLock == 0u) {
        CurrentDoorState = DOOR_OPEN;
    }
}

/*
 * ForceServo90Deg (v15 Unified Architecture Feature):
 * Centralized structural safety function utilized by every stop/emergency path.
 * Enforces: Latch State Flags -> Write Registers -> Block for Physical Transit -> Turn Outputs Off.
 */
static void ForceServo90Deg(void)
{
    CPU_SR_ALLOC();

    /* 1. Atomically set state flags first to preempt competing animations */
    CPU_CRITICAL_ENTER();
    ServoHardLock            = 1u;
    SystemRun                = DEF_FALSE;
    DoorHoldClosedUntilStart = DEF_TRUE;
    CurrentDoorState         = DOOR_LOCKED;
    CPU_CRITICAL_EXIT();

    /* 2. Execute instantaneous raw registers write to target angle */
    ServoClose90Deg_hw();

    /* 3. Block calling thread context to wait for physical travel duration */
    ServoWaitFor90Deg();

    /* 4. Strip power away from electrical outputs safely AFTER mechanical close validation */
    AllConveyorsOff();
    BuzzerOff();
}

static void ServoMoveSmooth(uint32_t from_us, uint32_t to_us,
        uint32_t steps,  uint32_t step_delay_ms)
{
    OS_ERR   err;
    uint32_t i;
    int32_t  delta;
    int32_t  current_us = (int32_t)from_us;
    int32_t  step_size;

    if (steps == 0u) {
        if (ServoHardLock == 0u) {
            ServoSetPulseUs(to_us);
        } else {
            ServoClose90Deg_hw();
        }
        return;
    }

    delta     = (int32_t)to_us - (int32_t)from_us;
    step_size = delta / (int32_t)steps;

    for (i = 0u; i < steps; i++) {
        if (ServoHardLock != 0u) {
            ServoClose90Deg_hw();
            return;
        }

        current_us += step_size;
        ServoSetPulseUs((uint32_t)current_us);
        OSTimeDlyHMSM(0u, 0u, 0u, step_delay_ms, OS_OPT_TIME_HMSM_STRICT, &err);
    }

    if (ServoHardLock != 0u) {
        ServoClose90Deg_hw();
    } else {
        ServoSetPulseUs(to_us);
    }
}

/* =========================================================
 * StopSystemAndCloseDoor (v15 Wrapped)
 * =========================================================*/
static void StopSystemAndCloseDoor(void)
{
    OS_ERR err;

    /* Redirect directly through internal unified safety routine */
    ForceServo90Deg();

    OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
    OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
}

/* =========================================================
 * Emergency ISR + task
 * =========================================================*/

static void AppEmergencyISR(void)
{
    OS_ERR err;
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line1);
        OSSemPost(&EmergencySem, OS_OPT_POST_1, &err);
    }
}

static void AppTaskEmergency(void *p_arg)
{
    OS_ERR err;
    CPU_TS ts;
    CPU_SR_ALLOC();
    (void)p_arg;

    while (DEF_TRUE) {
        OSSemPend(&EmergencySem, 0u, OS_OPT_PEND_BLOCKING, &ts, &err);
        if (err != OS_ERR_NONE) { continue; }

        OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);

        if ((GPIO_ReadInputDataBit(GPIOC, ESTOP_BUTTON_PIN) == Bit_SET) ||
            (SensorEmergencyRequest == DEF_TRUE) ||
            ((CurrentPersonState == PERSON_NEAR) && (MaintenanceInProgress == DEF_FALSE))) {

            /* Invoke unified structural function to handle safety execution ordering */
            ForceServo90Deg();

            CPU_CRITICAL_ENTER();
            EmergencyActive = DEF_TRUE;
            SensorEmergencyRequest = DEF_FALSE;
            ConveyorAState   = SYS_STATE_EMERGENCY;
            ConveyorBState   = SYS_STATE_EMERGENCY;
            OverallState     = SYS_STATE_EMERGENCY;
            CPU_CRITICAL_EXIT();

            GPIO_ResetBits(GPIOC, RGB_ALL_MASK);
            OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);

            UsartPrint("\r\n==================================================\r\n");
            if ((CurrentPersonState == PERSON_NEAR) && (MaintenanceInProgress == DEF_FALSE)) {
                UsartPrint(" !!! PERSON <= 8 cm DETECTED - EMERGENCY STOP !!!\r\n");
            } else if (GPIO_ReadInputDataBit(GPIOC, ESTOP_BUTTON_PIN) == Bit_SET) {
                UsartPrint(" !!! EMERGENCY BUTTON PRESSED - EMERGENCY STOP !!!\r\n");
            } else {
                UsartPrint(" !!! SENSOR/HEALTH EMERGENCY STOP !!!\r\n");
            }
            UsartPrint(" Servo physically closed to 90 deg. All outputs OFF.\r\n");
            UsartPrint(" Release PC1 (if pressed) then type 'clear'.\r\n");
            UsartPrint("==================================================\r\n> ");
        }
    }
}

/* =========================================================
 * Start / pause button task
 * =========================================================*/

static void AppTaskButton(void *p_arg)
{
    OS_ERR err;
    CPU_BOOLEAN start_previous = DEF_FALSE;
    CPU_BOOLEAN start_current;
    CPU_BOOLEAN maint_previous = DEF_FALSE;
    CPU_BOOLEAN maint_current;
    CPU_SR_ALLOC();
    (void)p_arg;

    while (DEF_TRUE) {
        start_current = (GPIO_ReadInputDataBit(GPIOC, START_BUTTON_PIN) == Bit_SET) ? DEF_TRUE : DEF_FALSE;
        maint_current = MaintButtonIsPressed();

        if ((start_previous == DEF_FALSE) && (start_current == DEF_TRUE)) {
            OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);

            if (GPIO_ReadInputDataBit(GPIOC, START_BUTTON_PIN) == Bit_SET) {
                if (EmergencyActive == DEF_FALSE) {
                    if (SystemRun == DEF_TRUE) {
                        StopSystemAndCloseDoor();
                        UsartPrint("\r\n[EVENT] Conveyors paused. Servo physically at 90 deg.\r\n> ");
                    } else {
                        CPU_CRITICAL_ENTER();
                        ServoHardLock            = 0u;
                        DoorHoldClosedUntilStart = DEF_FALSE;
                        CurrentDoorState         = DOOR_LOCKED;
                        CPU_CRITICAL_EXIT();

                        ServoOpen0Deg_hw();
                        ServoWaitFor0Deg();

                        CPU_CRITICAL_ENTER();
                        SystemRun = DEF_TRUE;
                        CPU_CRITICAL_EXIT();

                        OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
                        OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
                        UsartPrint("\r\n[EVENT] Servo physically open. Conveyors started.\r\n> ");
                    }
                } else {
                    UsartPrint("\r\n[ERROR] Emergency is latched. Type 'clear'.\r\n> ");
                }

                while (GPIO_ReadInputDataBit(GPIOC, START_BUTTON_PIN) == Bit_SET) {
                    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
                }
            }
        }

        if ((maint_previous == DEF_FALSE) && (maint_current == DEF_TRUE)) {
            OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);

            if (MaintButtonIsPressed() == DEF_TRUE) {
                UsartPrint("\r\n[BUTTON] Maintenance button pressed.\r\n");
                ApplySelectedMaintenance();
                UsartPrint("> ");

                while (MaintButtonIsPressed() == DEF_TRUE) {
                    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
                }
            }
        }

        start_previous = start_current;
        maint_previous = maint_current;
        OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 * ADXL345 sensor task
 * =========================================================*/

static void AppTaskSensor(void *p_arg)
{
    OS_ERR err;
    uint8_t device_id;
    int16_t x, y, z;
    int16_t previous_x = 0, previous_y = 0;
    CPU_BOOLEAN first_sample = DEF_TRUE;
    uint32_t filtered_a = 0u, filtered_b = 0u;
    uint32_t raw_a, raw_b;
    char message[160];
    CPU_SR_ALLOC();
    (void)p_arg;

    OSTimeDlyHMSM(0u, 0u, 0u, 200u, OS_OPT_TIME_HMSM_STRICT, &err);

    while (DEF_TRUE) {
        device_id = 0u;
        if ((ADXLReadReg(ADXL_REG_DEVID, &device_id) == DEF_TRUE) && (device_id == ADXL_EXPECTED_DEVID)) { break; }

        snprintf(message, sizeof(message), "[ADXL] Not detected (ID=0x%02X). Retrying...\r\n", device_id);
        UsartPrint(message);
        SetupI2C1();
        OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
    }

    if ((ADXLWriteReg(ADXL_REG_DATA_FORMAT, 0x09u) == DEF_FALSE) || (ADXLWriteReg(ADXL_REG_POWER_CTL, 0x08u) == DEF_FALSE)) {
        UsartPrint("[ADXL] Configuration failed.\r\n");
    } else {
        UsartPrint("[ADXL] Ready.\r\n> ");
    }

    while (DEF_TRUE) {
        if (ADXLReadXYZ(&x, &y, &z) == DEF_FALSE) {
            UsartPrint("[ADXL] Read failed; resetting I2C.\r\n");
            SetupI2C1();
            OSTimeDlyHMSM(0u, 0u, 0u, SENSOR_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
            continue;
        }

        if (first_sample == DEF_TRUE) {
            previous_x = x; previous_y = y;
            first_sample = DEF_FALSE;
            raw_a = 0u; raw_b = 0u;
        } else {
            raw_a = (uint32_t)abs((int)x - (int)previous_x);
            raw_b = (uint32_t)abs((int)y - (int)previous_y);
            previous_x = x; previous_y = y;
        }

        if (raw_a > 600u) raw_a = 600u;
        if (raw_b > 600u) raw_b = 600u;
        filtered_a = ((filtered_a * 3u) + raw_a) / 4u;
        filtered_b = ((filtered_b * 3u) + raw_b) / 4u;

        CPU_CRITICAL_ENTER();
        LatestSensorData.X     = x;
        LatestSensorData.Y     = y;
        LatestSensorData.Z     = z;
        LatestSensorData.VibA  = filtered_a;
        LatestSensorData.VibB  = filtered_b;
        LatestSensorData.Valid = DEF_TRUE;
        CPU_CRITICAL_EXIT();

        OSSemPost(&SensorDataSem, OS_OPT_POST_1, &err);
        OSTimeDlyHMSM(0u, 0u, 0u, SENSOR_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 * Timer / microsecond delay
 * =========================================================*/

static void Timer_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer_init;
    uint32_t timer_clk;

    RCC_GetClocksFreq(&clocks);
    SysClockHz = clocks.HCLK_Frequency;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timer_clk = clocks.PCLK1_Frequency * 2u;
    } else {
        timer_clk = clocks.PCLK1_Frequency;
    }

    timer_init.TIM_Prescaler     = (timer_clk / 1000000u) - 1u;
    timer_init.TIM_Period        = 0xFFFFu;
    timer_init.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_init.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &timer_init);
    TIM_Cmd(TIM7, ENABLE);
}

static void DelayUs(uint32_t us)
{
    uint16_t start;
    while (us > 0u) {
        uint32_t chunk = (us > 60000u) ? 60000u : us;
        start = (uint16_t)TIM7->CNT;
        while ((uint16_t)(TIM7->CNT - start) < (uint16_t)chunk) { }
        us -= chunk;
    }
}

/* =========================================================
 * HC-SR04 ultrasonic ranging
 * =========================================================*/

static uint32_t UltraMeasureCm(void)
{
    uint16_t start;
    uint16_t elapsed;

    GPIO_ResetBits(ULTRA_GPIO, ULTRA_TRIG_PIN);
    DelayUs(2u);
    GPIO_SetBits(ULTRA_GPIO, ULTRA_TRIG_PIN);
    DelayUs(10u);
    GPIO_ResetBits(ULTRA_GPIO, ULTRA_TRIG_PIN);

    start = (uint16_t)TIM7->CNT;
    while (GPIO_ReadInputDataBit(ULTRA_GPIO, ULTRA_ECHO_PIN) == Bit_RESET) {
        if ((uint16_t)(TIM7->CNT - start) > 25000u) { return ULTRA_MAX_RANGE_CM; }
    }

    start = (uint16_t)TIM7->CNT;
    while (GPIO_ReadInputDataBit(ULTRA_GPIO, ULTRA_ECHO_PIN) == Bit_SET) {
        if ((uint16_t)(TIM7->CNT - start) > 25000u) { return ULTRA_MAX_RANGE_CM; }
    }

    elapsed = (uint16_t)(TIM7->CNT - start);
    if (elapsed == 0u) { return 0u; }
    return ((uint32_t)elapsed + 29u) / 58u;
}

/* =========================================================
 * Potentiometer / Maintenance helpers
 * =========================================================*/

static uint32_t PotReadRaw(void)
{
    uint32_t timeout = 50000u;
    ADC_RegularChannelConfig(POT_ADC, POT_ADC_CHANNEL, 1u, ADC_SampleTime_144Cycles);
    ADC_SoftwareStartConv(POT_ADC);
    while (ADC_GetFlagStatus(POT_ADC, ADC_FLAG_EOC) == RESET) {
        if (timeout-- == 0u) { return PotAdcRaw; }
    }
    return (uint32_t)ADC_GetConversionValue(POT_ADC);
}

static uint8_t PotRawToMaintenanceLevel(uint32_t raw)
{
    uint8_t level;
    if (raw > POT_ADC_MAX) { raw = POT_ADC_MAX; }
    level = (uint8_t)((raw * MAINT_LEVEL_COUNT) / (POT_ADC_MAX + 1u)) + 1u;
    if (level < 1u) { level = 1u; }
    if (level > MAINT_LEVEL_COUNT) { level = MAINT_LEVEL_COUNT; }
    return level;
}

static const MAINTENANCE_LEVEL_INFO *GetMaintenanceInfo(uint8_t level)
{
    if (level < 1u) { level = 1u; }
    if (level > MAINT_LEVEL_COUNT) { level = MAINT_LEVEL_COUNT; }
    return &MaintenanceTable[level - 1u];
}

static void UpdateFactoryEconomy(void)
{
    uint32_t product_total;
    uint32_t profit;
    int32_t  net;
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
    product_total = ProductCountA + ProductCountB;
    profit        = product_total * PROFIT_PER_PRODUCT;
    FactoryProfit = profit;
    net           = (int32_t)profit - (int32_t)TotalMaintenanceCost;
    FactoryNetProfit = net;
    CPU_CRITICAL_EXIT();
}

static void ApplySelectedMaintenance(void)
{
    OS_ERR err;
    const MAINTENANCE_LEVEL_INFO *info;
    uint8_t  level;
    uint32_t old_health;
    uint32_t new_health;
    char message[260];
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
    level      = SelectedMaintLevel;
    old_health = MachineHealth;
    CPU_CRITICAL_EXIT();

    info = GetMaintenanceInfo(level);

    if (old_health > MAINT_ALLOWED_HEALTH) {
        snprintf(message, sizeof(message), "Maintenance denied: MachineHealth=%lu. Allowed only when <= %lu.\r\n", (unsigned long)old_health, (unsigned long)MAINT_ALLOWED_HEALTH);
        UsartPrint(message);
        return;
    }

    if (GPIO_ReadInputDataBit(GPIOC, ESTOP_BUTTON_PIN) == Bit_SET) {
        UsartPrint("Maintenance denied: physical E-STOP is pressed. Release PC1 first.\r\n");
        return;
    }

    if (MaintenanceInProgress == DEF_TRUE) {
        UsartPrint("Maintenance denied: already in progress.\r\n");
        return;
    }

    if (CurrentPersonState == PERSON_NEAR) {
        UsartPrint("[MAINT] Person near sensor, but maintenance allowed (conveyors stop).\r\n");
    }

    /* Standard centralized structural execution bottleneck */
    ForceServo90Deg();

    CPU_CRITICAL_ENTER();
    EmergencyActive            = DEF_FALSE;
    SensorEmergencyRequest     = DEF_FALSE;
    MaintenanceInProgress      = DEF_TRUE;
    CPU_CRITICAL_EXIT();

    OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
    OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);

    snprintf(message, sizeof(message), "[MAINT] Starting %s. cost=%lu, health +%lu, downtime=%lu sec.\r\n", info->Name, (unsigned long)info->Cost, (unsigned long)info->HealthGain, (unsigned long)info->DowntimeSec);
    UsartPrint(message);

    {
        uint32_t remaining_sec;
        for (remaining_sec = info->DowntimeSec; remaining_sec > 0u; remaining_sec--) {
            snprintf(message, sizeof(message), "[MAINT] %s countdown: %lu sec remaining.\r\n", info->Name, (unsigned long)remaining_sec);
            UsartPrint(message);
            OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
        }
    }

    CPU_CRITICAL_ENTER();
    old_health = MachineHealth;
    if ((MachineHealth + info->HealthGain) > MACHINE_HEALTH_MAX) {
        MachineHealth = MACHINE_HEALTH_MAX;
    } else {
        MachineHealth += info->HealthGain;
    }
    new_health = MachineHealth;
    TotalMaintenanceCost += info->Cost;
    MaintenanceInProgress    = DEF_FALSE;
    DoorHoldClosedUntilStart = DEF_TRUE;
    CurrentDoorState         = DOOR_LOCKED;
    CPU_CRITICAL_EXIT();

    UpdateFactoryEconomy();

    snprintf(message, sizeof(message), "[MAINT] Finished. Health %lu -> %lu | profit=%ld cost=%ld net=%ld.\r\nGate at 90 deg. Type 'start' to physically reopen and restart.\r\n", (unsigned long)old_health, (unsigned long)new_health, (long)FactoryProfit, (long)TotalMaintenanceCost, (long)FactoryNetProfit);
    UsartPrint(message);

    OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
    OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
}

static void PrintEconomy(void)
{
    char message[360];
    const MAINTENANCE_LEVEL_INFO *info;
    CPU_SR_ALLOC();

    UpdateFactoryEconomy();

    CPU_CRITICAL_ENTER();
    info = GetMaintenanceInfo(SelectedMaintLevel);
    snprintf(message, sizeof(message),
             "runtime=%lu sec\r\n"
             "products: A=%lu B=%lu total=%lu\r\n"
             "profit=%ld cost=%ld net=%ld\r\n"
             "MachineHealth=%lu / %lu\r\n"
             "maintenance_allowed=%s (condition: MachineHealth <= %lu)\r\n"
             "pot_adc=%lu selected=%s\r\n"
             "selected_cost=%lu selected_health_gain=%lu selected_downtime=%lu sec\r\n",
             (unsigned long)FactoryRunTimeSec, (unsigned long)ProductCountA, (unsigned long)ProductCountB, (unsigned long)(ProductCountA + ProductCountB), (long)FactoryProfit, (long)TotalMaintenanceCost, (long)FactoryNetProfit, (unsigned long)MachineHealth, (unsigned long)MACHINE_HEALTH_MAX, (MachineHealth <= MAINT_ALLOWED_HEALTH) ? "YES" : "NO", (unsigned long)MAINT_ALLOWED_HEALTH, (unsigned long)PotAdcRaw, info->Name, (unsigned long)info->Cost, (unsigned long)info->HealthGain, (unsigned long)info->DowntimeSec);
    CPU_CRITICAL_EXIT();

    UsartPrint(message);
}

/* =========================================================
 * Physical maintenance button helper
 * =========================================================*/

static uint8_t MaintButtonReadWithPull(GPIOPuPd_TypeDef pull)
{
    GPIO_InitTypeDef gpio;
    volatile uint32_t settle;

    gpio.GPIO_Pin   = MAINT_BUTTON_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = pull;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    for (settle = 0u; settle < 200u; settle++) { }

    return GPIO_ReadInputDataBit(GPIOC, MAINT_BUTTON_PIN);
}

static CPU_BOOLEAN MaintButtonIsPressed(void)
{
    uint8_t read_with_pulldown;
    uint8_t read_with_pullup;

    read_with_pulldown = MaintButtonReadWithPull(GPIO_PuPd_DOWN);
    read_with_pullup   = MaintButtonReadWithPull(GPIO_PuPd_UP);

    if (MaintButtonIdleSaved == DEF_FALSE) {
        MaintButtonIdlePulldown = read_with_pulldown;
        MaintButtonIdlePullup   = read_with_pullup;
        MaintButtonIdleSaved    = DEF_TRUE;
        return DEF_FALSE;
    }

    if ((read_with_pulldown != MaintButtonIdlePulldown) || (read_with_pullup != MaintButtonIdlePullup)) {
        return DEF_TRUE;
    }

    return DEF_FALSE;
}

static void PrintMaintButtonDebug(void)
{
    uint8_t read_with_pulldown;
    uint8_t read_with_pullup;
    CPU_BOOLEAN pressed;
    char message[300];

    read_with_pulldown = MaintButtonReadWithPull(GPIO_PuPd_DOWN);
    read_with_pullup   = MaintButtonReadWithPull(GPIO_PuPd_UP);

    if (MaintButtonIdleSaved == DEF_FALSE) {
        MaintButtonIdlePulldown = read_with_pulldown;
        MaintButtonIdlePullup   = read_with_pullup;
        MaintButtonIdleSaved    = DEF_TRUE;
    }

    pressed = ((read_with_pulldown != MaintButtonIdlePulldown) || (read_with_pullup != MaintButtonIdlePullup)) ? DEF_TRUE : DEF_FALSE;

    snprintf(message, sizeof(message), "PC0 button debug: idle_PD=%s idle_PU=%s now_PD=%s now_PU=%s interpreted=%s\r\n", (MaintButtonIdlePulldown == Bit_SET) ? "HIGH" : "LOW", (MaintButtonIdlePullup == Bit_SET) ? "HIGH" : "LOW", (read_with_pulldown == Bit_SET) ? "HIGH" : "LOW", (read_with_pullup == Bit_SET) ? "HIGH" : "LOW", (pressed == DEF_TRUE) ? "PRESSED" : "NOT PRESSED");
    UsartPrint(message);
}

/* =========================================================
 * Ultrasonic task
 * =========================================================*/

static void AppTaskUltrasonic(void *p_arg)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    uint32_t       dist_cm;
    PERSON_STATE   new_ps;
    PERSON_STATE   old_ps;
    uint32_t       near_count = 0u;
    uint32_t       warn_count = 0u;
    uint32_t       none_count = 0u;
    char           message[80];

    (void)p_arg;

    OSTimeDlyHMSM(0u, 0u, 0u, 500u, OS_OPT_TIME_HMSM_STRICT, &err);
    UsartPrint("[ULTRA] HC-SR04 started on PA2(TRIG) PA3(ECHO).\r\n> ");

    while (DEF_TRUE) {
        dist_cm = UltraMeasureCm();

        CPU_CRITICAL_ENTER();
        LastDistanceCm = dist_cm;
        old_ps = CurrentPersonState;
        CPU_CRITICAL_EXIT();

        if ((dist_cm < ULTRA_VALID_MIN_CM) || (dist_cm >= ULTRA_MAX_RANGE_CM)) {
            new_ps = old_ps;
            near_count = 0u;
            warn_count = 0u;
        } else if (dist_cm <= ULTRA_NEAR_CM) {
            near_count++;
            warn_count = 0u;
            none_count = 0u;
            new_ps = (near_count >= ULTRA_ENTER_NEAR_COUNT) ? PERSON_NEAR : old_ps;
        } else if (dist_cm < ULTRA_WARN_CM) {
            warn_count++;
            near_count = 0u;
            none_count = 0u;
            new_ps = (warn_count >= ULTRA_ENTER_WARN_COUNT) ? PERSON_WARN : old_ps;
        } else {
            none_count++;
            near_count = 0u;
            warn_count = 0u;
            new_ps = (none_count >= ULTRA_LEAVE_WARN_COUNT) ? PERSON_NONE : old_ps;
        }

        if (new_ps != old_ps) {
            CPU_CRITICAL_ENTER();
            PreviousPersonState = old_ps;
            CurrentPersonState  = new_ps;
            CPU_CRITICAL_EXIT();

            snprintf(message, sizeof(message), "\r\n[PERSON] %s -> %s (dist=%lu cm)\r\n> ", PersonStateToString(old_ps), PersonStateToString(new_ps), (unsigned long)dist_cm);
            UsartPrint(message);

            if (new_ps == PERSON_NEAR) {
                if (MaintenanceInProgress == DEF_FALSE) {
                    /* Central structural safety function called instantly here */
                    ForceServo90Deg();
                    OSSemPost(&EmergencySem, OS_OPT_POST_1, &err);
                }
            } else if (new_ps == PERSON_WARN) {
                CPU_CRITICAL_ENTER();
                if (OverallState < SYS_STATE_WARNING) {
                    OverallState = SYS_STATE_WARNING;
                }
                CPU_CRITICAL_EXIT();
                OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
                OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
            } else {
                OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
                OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
            }
        }

        OSTimeDlyHMSM(0u, 0u, 0u, ULTRA_MEASURE_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 * Potentiometer + Maintenance / Economy task
 * =========================================================*/

static void AppTaskMaintenance(void *p_arg)
{
    OS_ERR err;
    uint32_t raw;
    uint8_t  level;
    uint8_t  old_level = 0u;
    uint32_t sample_ticks = 0u;
    uint32_t normal_wear_seconds = 0u;
    uint32_t old_health;
    uint32_t damage;
    SYSTEM_STATE state;
    CPU_SR_ALLOC();
    (void)p_arg;

    OSTimeDlyHMSM(0u, 0u, 0u, 300u, OS_OPT_TIME_HMSM_STRICT, &err);
    UsartPrint("[POT] Maintenance selector started on PA4 (ADC1 CH4).\r\n> ");

    while (DEF_TRUE) {
        raw   = PotReadRaw();
        level = PotRawToMaintenanceLevel(raw);

        CPU_CRITICAL_ENTER();
        PotAdcRaw          = raw;
        SelectedMaintLevel = level;
        CPU_CRITICAL_EXIT();

        if (level != old_level) { old_level = level; }

        sample_ticks++;
        if (sample_ticks >= (1000u / POT_SAMPLE_PERIOD_MS)) {
            sample_ticks = 0u;
            UpdateFactoryEconomy();

            if ((SystemRun == DEF_TRUE) && (EmergencyActive == DEF_FALSE) && (MaintenanceInProgress == DEF_FALSE)) {
                FactoryRunTimeSec++;
                state = GetOverallState();
                damage = 0u;

                if (state == SYS_STATE_DANGER) {
                    damage = 3u;
                    normal_wear_seconds = 0u;
                } else if ((state == SYS_STATE_WARNING) || (state == SYS_STATE_RECOVERY) || (CurrentPersonState == PERSON_WARN)) {
                    damage = 1u;
                    normal_wear_seconds = 0u;
                } else {
                    normal_wear_seconds++;
                    if (normal_wear_seconds >= 10u) {
                        damage = 1u;
                        normal_wear_seconds = 0u;
                    }
                }

                if (damage > 0u) {
                    CPU_CRITICAL_ENTER();
                    old_health = MachineHealth;
                    if (MachineHealth > damage) {
                        MachineHealth -= damage;
                    } else {
                        MachineHealth = MACHINE_HEALTH_MIN;
                    }
                    CPU_CRITICAL_EXIT();

                    if ((old_health > MAINT_ALLOWED_HEALTH) && (MachineHealth <= MAINT_ALLOWED_HEALTH)) {
                        UsartPrint("\r\n[MAINT] MachineHealth <= 70. Maintenance is now allowed.\r\n> ");
                    }

                    if (MachineHealth == MACHINE_HEALTH_MIN) {
                        SensorEmergencyRequest = DEF_TRUE;
                        OSSemPost(&EmergencySem, OS_OPT_POST_1, &err);
                        UsartPrint("\r\n[MAINT] MachineHealth reached 0. Emergency stop requested.\r\n> ");
                    }
                }
            }
        }

        OSTimeDlyHMSM(0u, 0u, 0u, POT_SAMPLE_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 * Control task (vibration state machine)
 * =========================================================*/

static void AppTaskControl(void *p_arg)
{
    OS_ERR err;
    CPU_TS ts;
    SENSOR_DATA  sample;
    SYSTEM_STATE old_a, old_b, old_overall;
    SYSTEM_STATE new_a, new_b, new_overall;
    STATE_TRACKER tracker_a = {SYS_STATE_NORMAL, 0u, 0u, 0u, 0u};
    STATE_TRACKER tracker_b = {SYS_STATE_NORMAL, 0u, 0u, 0u, 0u};
    uint32_t extreme_a_count = 0u;
    uint32_t extreme_b_count = 0u;
    char message[200];
    CPU_SR_ALLOC();
    (void)p_arg;

    while (DEF_TRUE) {
        OSSemPend(&SensorDataSem, 0u, OS_OPT_PEND_BLOCKING, &ts, &err);
        if (err != OS_ERR_NONE) { continue; }

        CPU_CRITICAL_ENTER();
        sample      = LatestSensorData;
        old_a       = ConveyorAState;
        old_b       = ConveyorBState;
        old_overall = OverallState;
        CPU_CRITICAL_EXIT();

        if (sample.Valid == DEF_FALSE) { continue; }

        if (EmergencyActive == DEF_TRUE) {
            tracker_a.State = SYS_STATE_EMERGENCY;
            tracker_b.State = SYS_STATE_EMERGENCY;
            extreme_a_count = 0u;
            extreme_b_count = 0u;
            continue;
        }

        if (tracker_a.State == SYS_STATE_EMERGENCY) {
            memset(&tracker_a, 0, sizeof(tracker_a));
            tracker_a.State = SYS_STATE_NORMAL;
        }
        if (tracker_b.State == SYS_STATE_EMERGENCY) {
            memset(&tracker_b, 0, sizeof(tracker_b));
            tracker_b.State = SYS_STATE_NORMAL;
        }

        if (SystemRun == DEF_TRUE) {
            extreme_a_count = (sample.VibA >= VIB_EMERGENCY_UP) ? (extreme_a_count + 1u) : 0u;
            extreme_b_count = (sample.VibB >= VIB_EMERGENCY_UP) ? (extreme_b_count + 1u) : 0u;

            if ((extreme_a_count >= EMERGENCY_CONFIRM_SAMPLES) || (extreme_b_count >= EMERGENCY_CONFIRM_SAMPLES)) {
                SensorEmergencyRequest = DEF_TRUE;
                extreme_a_count = 0u;
                extreme_b_count = 0u;
                OSSemPost(&EmergencySem, OS_OPT_POST_1, &err);
                continue;
            }
        } else {
            extreme_a_count = 0u;
            extreme_b_count = 0u;
        }

        new_a = UpdateVibrationState(&tracker_a, sample.VibA);
        new_b = UpdateVibrationState(&tracker_b, sample.VibB);
        new_overall = (new_a > new_b) ? new_a : new_b;

        if ((CurrentPersonState == PERSON_WARN) && (new_overall < SYS_STATE_WARNING)) {
            new_overall = SYS_STATE_WARNING;
        }

        CPU_CRITICAL_ENTER();
        if (EmergencyActive == DEF_FALSE) {
            ConveyorAState   = new_a;
            ConveyorBState   = new_b;
            OverallState     = new_overall;
            ConveyorADelayMs = StateToDelay(new_a);
            ConveyorBDelayMs = StateToDelay(new_b);
        }
        CPU_CRITICAL_EXIT();

        if (new_overall != old_overall) {
            OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
            OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
        }

        if ((new_a != old_a) || (new_b != old_b)) {
            snprintf(message, sizeof(message), "\r\n[STATE] A: %s->%s (vib=%lu), B: %s->%s (vib=%lu) overall=%s person=%s\r\n> ", StateToString(old_a), StateToString(new_a), (unsigned long)sample.VibA, StateToString(old_b), StateToString(new_b), (unsigned long)sample.VibB, StateToString(new_overall), PersonStateToString(CurrentPersonState));
            UsartPrint(message);
        }
    }
}

static SYSTEM_STATE UpdateVibrationState(STATE_TRACKER *tracker, uint32_t vibration)
{
    if (tracker->HoldCount > 0u) { tracker->HoldCount--; }

    switch (tracker->State) {
        case SYS_STATE_NORMAL:
            tracker->SafeCount     = 0u;
            tracker->RecoveryCount = 0u;
            if (vibration >= VIB_WARN_UP) {
                tracker->HighCount++;
                if (tracker->HighCount >= WARN_CONFIRM_SAMPLES) {
                    tracker->State     = SYS_STATE_WARNING;
                    tracker->HoldCount = WARNING_MIN_SAMPLES;
                    tracker->HighCount = 0u;
                }
            } else {
                tracker->HighCount = 0u;
            }
            break;

        case SYS_STATE_WARNING:
            if (vibration >= VIB_DANGER_UP) {
                tracker->HighCount++;
                tracker->SafeCount = 0u;
                if ((tracker->HoldCount == 0u) && (tracker->HighCount >= DANGER_CONFIRM_SAMPLES)) {
                    tracker->State     = SYS_STATE_DANGER;
                    tracker->HoldCount = DANGER_MIN_SAMPLES;
                    tracker->HighCount = 0u;
                }
            } else if (vibration < VIB_WARN_DOWN) {
                tracker->SafeCount++;
                tracker->HighCount = 0u;
                if ((tracker->HoldCount == 0u) && (tracker->SafeCount >= SAFE_CONFIRM_SAMPLES)) {
                    tracker->State     = SYS_STATE_RECOVERY;
                    tracker->HoldCount = RECOVERY_MIN_SAMPLES;
                    tracker->SafeCount = 0u;
                }
            } else {
                tracker->HighCount = 0u;
                tracker->SafeCount = 0u;
            }
            break;

        case SYS_STATE_RECOVERY:
            if (tracker->HoldCount > 0u) { break; }
            if (vibration >= VIB_DANGER_UP) {
                tracker->HighCount++;
                tracker->SafeCount = 0u;
                if (tracker->HighCount >= DANGER_CONFIRM_SAMPLES) {
                    tracker->State     = SYS_STATE_DANGER;
                    tracker->HoldCount = DANGER_MIN_SAMPLES;
                    tracker->HighCount = 0u;
                }
            } else if (vibration < VIB_WARN_DOWN) {
                tracker->SafeCount++;
                tracker->HighCount = 0u;
                if (tracker->SafeCount >= SAFE_CONFIRM_SAMPLES) {
                    tracker->State     = SYS_STATE_NORMAL;
                    tracker->SafeCount = 0u;
                }
            } else if (vibration >= VIB_WARN_UP) {
                tracker->HighCount++;
                tracker->SafeCount = 0u;
                if (tracker->HighCount >= WARN_CONFIRM_SAMPLES) {
                    tracker->State     = SYS_STATE_WARNING;
                    tracker->HoldCount = WARNING_MIN_SAMPLES;
                    tracker->HighCount = 0u;
                }
            } else {
                tracker->HighCount = 0u;
                tracker->SafeCount = 0u;
            }
            break;

        case SYS_STATE_DANGER:
            if (tracker->HoldCount > 0u) { break; }
            if (vibration < VIB_DANGER_DOWN) {
                tracker->SafeCount++;
                if (tracker->SafeCount >= SAFE_CONFIRM_SAMPLES) {
                    tracker->State         = SYS_STATE_RECOVERY;
                    tracker->HoldCount     = RECOVERY_MIN_SAMPLES;
                    tracker->SafeCount     = 0u;
                    tracker->RecoveryCount = 0u;
                }
            } else {
                tracker->SafeCount = 0u;
            }
            break;

        case SYS_STATE_EMERGENCY:
        default:
            tracker->State         = SYS_STATE_NORMAL;
            tracker->HighCount     = 0u;
            tracker->SafeCount     = 0u;
            tracker->HoldCount     = 0u;
            tracker->RecoveryCount = 0u;
            break;
    }

    return tracker->State;
}

/* =========================================================
 * Conveyor A and B tasks
 * =========================================================*/

static void AppTaskConveyorA(void *p_arg)
{
    static const uint16_t pins[4] = { GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3 };
    uint32_t index = 0u;
    SYSTEM_STATE state;
    uint32_t delay_ms;
    OS_ERR err;
    (void)p_arg;

    while (DEF_TRUE) {
        state    = ConveyorAState;
        delay_ms = ConveyorADelayMs;

        if ((SystemRun == DEF_FALSE) || (EmergencyActive == DEF_TRUE) || (state == SYS_STATE_EMERGENCY)) {
            GPIO_ResetBits(GPIOB, CONVEYOR_A_MASK);
            OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);
            continue;
        }

        GPIO_ResetBits(GPIOB, CONVEYOR_A_MASK);
        GPIO_SetBits(GPIOB, pins[index]);

        if (ConveyorResponsiveDelay(DEF_TRUE, state, delay_ms) == DEF_FALSE) {
            GPIO_ResetBits(GPIOB, CONVEYOR_A_MASK);
            continue;
        }

        GPIO_ResetBits(GPIOB, pins[index]);
        index = (index + 1u) % 4u;
        ProductCountA++;
    }
}

static void AppTaskConveyorB(void *p_arg)
{
    static const uint16_t pins[4] = { GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7 };
    uint32_t index = 0u;
    SYSTEM_STATE state;
    uint32_t delay_ms;
    OS_ERR err;
    (void)p_arg;

    while (DEF_TRUE) {
        state    = ConveyorBState;
        delay_ms = ConveyorBDelayMs;

        if ((SystemRun == DEF_FALSE) || (EmergencyActive == DEF_TRUE) || (state == SYS_STATE_EMERGENCY)) {
            GPIO_ResetBits(GPIOB, CONVEYOR_B_MASK);
            OSTimeDlyHMSM(0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &err);
            continue;
        }

        GPIO_ResetBits(GPIOB, CONVEYOR_B_MASK);
        GPIO_SetBits(GPIOB, pins[index]);

        if (ConveyorResponsiveDelay(DEF_FALSE, state, delay_ms) == DEF_FALSE) {
            GPIO_ResetBits(GPIOB, CONVEYOR_B_MASK);
            continue;
        }

        GPIO_ResetBits(GPIOB, pins[index]);
        index = (index + 1u) % 4u;
        ProductCountB++;
    }
}

static CPU_BOOLEAN ConveyorResponsiveDelay(CPU_BOOLEAN conveyor_a, SYSTEM_STATE expected_state, uint32_t delay_ms)
{
    OS_ERR err;
    uint32_t remaining = delay_ms;
    uint32_t part;
    SYSTEM_STATE current;

    while (remaining > 0u) {
        current = (conveyor_a == DEF_TRUE) ? ConveyorAState : ConveyorBState;

        if ((EmergencyActive == DEF_TRUE) || (SystemRun == DEF_FALSE) || (current != expected_state) || (current == SYS_STATE_EMERGENCY)) {
            return DEF_FALSE;
        }

        part = (remaining > 10u) ? 10u : remaining;
        OSTimeDlyHMSM(0u, 0u, 0u, part, OS_OPT_TIME_HMSM_STRICT, &err);
        remaining -= part;
    }
    return DEF_TRUE;
}

/* =========================================================
 * RGB task
 * =========================================================*/

static void AppTaskRgb(void *p_arg)
{
    OS_ERR err;
    CPU_TS ts;
    (void)p_arg;

    while (DEF_TRUE) {
        OSSemPend(&RgbUpdateSem, 0u, OS_OPT_PEND_BLOCKING, &ts, &err);
        if (err != OS_ERR_NONE) { continue; }

        if (EmergencyActive == DEF_TRUE) {
            SetRgb(SYS_STATE_EMERGENCY);
        } else if (SystemRun == DEF_FALSE) {
            GPIO_ResetBits(GPIOC, RGB_ALL_MASK);
        } else {
            SetRgb(GetOverallState());
        }
    }
}

static void SetRgb(SYSTEM_STATE state)
{
    GPIO_ResetBits(GPIOC, RGB_ALL_MASK);
    switch (state) {
        case SYS_STATE_NORMAL:
            GPIO_SetBits(GPIOC, RGB_GREEN_PIN);
            break;
        case SYS_STATE_WARNING:
            GPIO_SetBits(GPIOC, RGB_RED_PIN | RGB_BLUE_PIN);
            break;
        case SYS_STATE_RECOVERY:
            GPIO_SetBits(GPIOC, RGB_BLUE_PIN);
            break;
        case SYS_STATE_DANGER:
            GPIO_SetBits(GPIOC, RGB_RED_PIN);
            break;
        case SYS_STATE_EMERGENCY:
            break;
        default:
            break;
    }
}

/* =========================================================
 * Servo Motor Door Task
 * =========================================================*/

static void AppTaskServo(void *p_arg)
{
    OS_ERR       err;
    CPU_TS       ts;
    SYSTEM_STATE overall;
    DOOR_STATE   target_door;
    DOOR_STATE   old_door;
    char         message[80];
    (void)p_arg;

    ServoOpen0Deg_hw();
    CurrentDoorState = DOOR_OPEN;

    while (DEF_TRUE) {
        OSSemPend(&ServoUpdateSem, 0u, OS_OPT_PEND_BLOCKING, &ts, &err);
        if (err != OS_ERR_NONE) { continue; }

        if ((ServoHardLock != 0u) && (CurrentDoorState == DOOR_LOCKED)) {
            ServoClose90Deg_hw();
            OSTimeDlyHMSM(0u, 0u, 0u, 100u, OS_OPT_TIME_HMSM_STRICT, &err);
            continue;
        }

        old_door = CurrentDoorState;
        overall  = GetOverallState();

        if ((EmergencyActive == DEF_TRUE) || (overall == SYS_STATE_EMERGENCY) || (CurrentPersonState == PERSON_NEAR)) {
            target_door = DOOR_LOCKED;
        } else if ((MaintenanceInProgress == DEF_TRUE) || (DoorHoldClosedUntilStart == DEF_TRUE)) {
            target_door = DOOR_LOCKED;
        } else if (SystemRun == DEF_FALSE) {
            target_door = DOOR_LOCKED;
        } else if ((overall == SYS_STATE_DANGER) || (CurrentPersonState == PERSON_WARN && overall >= SYS_STATE_DANGER)) {
            target_door = DOOR_CLOSED;
        } else if ((overall == SYS_STATE_WARNING) || (CurrentPersonState == PERSON_WARN)) {
            target_door = DOOR_HALF;
        } else {
            target_door = DOOR_OPEN;
        }

        if (target_door == old_door) { continue; }

        switch (target_door) {
            case DOOR_OPEN:
                ServoMoveSmooth(SERVO_PULSE_CLOSED_US, SERVO_PULSE_OPEN_US, SERVO_STEPS, SERVO_STEP_DELAY_MS);
                if (ServoHardLock == 0u) { CurrentDoorState = DOOR_OPEN; }
                break;

            case DOOR_HALF:
                ServoMoveSmooth(SERVO_PULSE_OPEN_US, SERVO_PULSE_WARN_US, SERVO_STEPS, SERVO_STEP_DELAY_MS);
                if (ServoHardLock == 0u) { CurrentDoorState = DOOR_HALF; }
                break;

            case DOOR_CLOSED:
                ServoMoveSmooth((old_door == DOOR_HALF) ? SERVO_PULSE_WARN_US : SERVO_PULSE_OPEN_US, SERVO_PULSE_CLOSED_US, SERVO_STEPS / 2u, SERVO_STEP_DELAY_MS / 2u);
                if (ServoHardLock == 0u) { CurrentDoorState = DOOR_CLOSED; }
                break;

            case DOOR_LOCKED:
                {
                    CPU_SR_ALLOC();
                    CPU_CRITICAL_ENTER();
                    ServoHardLock    = 1u;
                    CurrentDoorState = DOOR_LOCKED;
                    CPU_CRITICAL_EXIT();
                }
                ServoClose90Deg_hw();
                ServoWaitFor90Deg();
                break;

            default:
                break;
        }

        if (CurrentDoorState != old_door) {
            snprintf(message, sizeof(message), "\r\n[DOOR] %s -> %s\r\n> ", DoorStateToString(old_door), DoorStateToString(CurrentDoorState));
            UsartPrint(message);
        }
    }
}

/* =========================================================
 * Buzzer task
 * =========================================================*/

static void AppTaskBuzzer(void *p_arg)
{
    OS_ERR err;
    SYSTEM_STATE state;
    (void)p_arg;
    BuzzerOff();

loop_entry:
    while (DEF_TRUE) {
        state = GetOverallState();

        if ((SystemRun == DEF_FALSE) && (state != SYS_STATE_EMERGENCY)) {
            BuzzerOff();
            OSTimeDlyHMSM(0u, 0u, 0u, 50u, OS_OPT_TIME_HMSM_STRICT, &err);
            continue;
        }

        switch (state) {
            case SYS_STATE_NORMAL:
                if (CurrentPersonState == PERSON_WARN) {
                    BuzzerTone(80u, state);
                    BuzzerSilence(1420u, state);
                } else {
                    BuzzerOff();
                    OSTimeDlyHMSM(0u, 0u, 0u, 100u, OS_OPT_TIME_HMSM_STRICT, &err);
                }
                break;
            case SYS_STATE_WARNING:
                BuzzerTone(100u, state);
                BuzzerSilence(1900u, state);
                break;
            case SYS_STATE_RECOVERY:
                BuzzerTone(100u, state);
                BuzzerSilence(900u, state);
                break;
            case SYS_STATE_DANGER:
                BuzzerTone(200u, state);
                BuzzerSilence(200u, state);
                break;
            case SYS_STATE_EMERGENCY:
                BuzzerOff();
                OSTimeDlyHMSM(0u, 0u, 0u, 50u, OS_OPT_TIME_HMSM_STRICT, &err);
                break;
            default:
                BuzzerOff();
                break;
        }
    }
}

static void BuzzerTone(uint32_t duration_ms, SYSTEM_STATE expected_state)
{
    uint32_t elapsed_ms;

#if BUZZER_IS_PASSIVE > 0u
    const uint32_t frequency_hz = 2000u;
    uint32_t half_period_us;
    uint32_t toggles_per_ms;
    uint32_t toggle;
    CPU_BOOLEAN buzzer_high = DEF_FALSE;
    volatile uint32_t loops;

    half_period_us   = 1000000u / (frequency_hz * 2u);
    toggles_per_ms   = (frequency_hz * 2u) / 1000u;
    if (toggles_per_ms == 0u) { toggles_per_ms = 1u; }

    for (elapsed_ms = 0u; elapsed_ms < duration_ms; elapsed_ms++) {
        if ((GetOverallState() != expected_state) || ((SystemRun == DEF_FALSE) && (expected_state != SYS_STATE_EMERGENCY))) {
            BuzzerOff();
            return;
        }
        for (toggle = 0u; toggle < toggles_per_ms; toggle++) {
            buzzer_high = (buzzer_high == DEF_TRUE) ? DEF_FALSE : DEF_TRUE;
            if (buzzer_high == DEF_TRUE) {
                GPIO_SetBits(GPIOC, BUZZER_PIN);
            } else {
                GPIO_ResetBits(GPIOC, BUZZER_PIN);
            }
            loops = (168u * half_period_us) / 4u;
            while (loops > 0u) { loops--; }
        }
    }
#else
    GPIO_SetBits(GPIOC, BUZZER_PIN);
    for (elapsed_ms = 0u; elapsed_ms < duration_ms; elapsed_ms++) {
        OS_ERR err;
        if ((GetOverallState() != expected_state) || ((SystemRun == DEF_FALSE) && (expected_state != SYS_STATE_EMERGENCY))) {
            break;
        }
        OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
#endif
    BuzzerOff();
}

static void BuzzerSilence(uint32_t duration_ms, SYSTEM_STATE expected_state)
{
    OS_ERR err;
    uint32_t remaining = duration_ms;
    uint32_t part;
    BuzzerOff();

    while (remaining > 0u) {
        if (GetOverallState() != expected_state) { return; }
        part = (remaining > 20u) ? 20u : remaining;
        OSTimeDlyHMSM(0u, 0u, 0u, part, OS_OPT_TIME_HMSM_STRICT, &err);
        remaining -= part;
    }
}

static void BuzzerOff(void)
{
    GPIO_ResetBits(GPIOC, BUZZER_PIN);
}

/* =========================================================
 * USART command task
 * =========================================================*/

static void AppTaskUsart(void *p_arg)
{
    OS_ERR err;
    char buffer[CMD_BUF_SIZE];
    uint32_t index = 0u;
    char character;
    (void)p_arg;

    while (DEF_TRUE) {
        if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET) {
            character = (char)USART_ReceiveData(USART3);

            if ((character == '\b') || (character == 0x7F)) {
                if (index > 0u) { index--; UsartPrint("\b \b"); }
            } else if ((character == '\r') || (character == '\n')) {
                UsartPrint("\r\n");
                buffer[index] = '\0';
                if (index > 0u) { HandleCommand(buffer); }
                index = 0u;
                UsartPrint("> ");
            } else if (index < (CMD_BUF_SIZE - 1u)) {
                buffer[index++] = character;
                UsartPutCharRaw(character);
            }
        }
        OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

static void NormalizeCommand(char *command)
{
    char *src;
    char *dst;
    int   last_space = 0;

    if (command == (char *)0) { return; }

    src = command;
    while ((*src == ' ') || (*src == '\t')) { src++; }

    dst = command;
    while (*src != '\0') {
        char c = *src++;
        if (c == '\t') { c = ' '; }
        if ((c >= 'A') && (c <= 'Z')) { c = (char)(c - 'A' + 'a'); }
        if (c == ' ') {
            if ((dst == command) || (last_space != 0)) { continue; }
            last_space = 1;
        } else {
            last_space = 0;
        }
        *dst++ = c;
    }
    while ((dst > command) && (*(dst - 1) == ' ')) { dst--; }
    *dst = '\0';
}

static void HandleCommand(char *command)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    NormalizeCommand(command);
    if (command[0] == '\0') { return; }

    if (strcmp(command, "start") == 0) {
        if (EmergencyActive == DEF_TRUE) {
            UsartPrint("Cannot start: emergency is latched. Type 'clear' first.\r\n");
        } else {
            CPU_CRITICAL_ENTER();
            ServoHardLock            = 0u;
            DoorHoldClosedUntilStart = DEF_FALSE;
            CurrentDoorState         = DOOR_LOCKED;
            CPU_CRITICAL_EXIT();

            UsartPrint("Opening door... please wait.\r\n");
            ServoOpen0Deg_hw();
            ServoWaitFor0Deg();

            CPU_CRITICAL_ENTER();
            SystemRun = DEF_TRUE;
            CPU_CRITICAL_EXIT();

            OSSemPost(&RgbUpdateSem,   OS_OPT_POST_1, &err);
            OSSemPost(&ServoUpdateSem, OS_OPT_POST_1, &err);
            UsartPrint("Servo physically open. Conveyors started.\r\n");
        }
        return;
    }

    if (strcmp(command, "stop") == 0) {
        StopSystemAndCloseDoor();
        UsartPrint("Servo physically at 90 deg. Conveyors stopped. Type 'start' to reopen.\r\n");
        return;
    }

    if (strcmp(command, "status") == 0) {
        PrintStatus();
        return;
    }

    if (strcmp(command, "count") == 0) {
        char out_str[96];
        sprintf(out_str, ">>> ConveyorA Count: %u , ConveyorB count: %u\r\n", (unsigned int)ProductCountA, (unsigned int)ProductCountB);
        UsartPrint(out_str);
        return;
    }

    if ((strcmp(command, "ultra") == 0) || (strcmp(command, "distance") == 0)) {
        char out_str[160];
        sprintf(out_str, "distance=%lu cm person=%s emergency<=%lu cm warning<%lu cm\r\n", (unsigned long)LastDistanceCm, PersonStateToString(CurrentPersonState), (unsigned long)ULTRA_NEAR_CM, (unsigned long)ULTRA_WARN_CM);
        UsartPrint(out_str);
        return;
    }

    if ((strcmp(command, "test emergency") == 0) || (strcmp(command, "emergency test") == 0)) {
        SensorEmergencyRequest = DEF_TRUE;
        OSSemPost(&EmergencySem, OS_OPT_POST_1, &err);
        UsartPrint("Emergency test requested.\r\n");
        return;
    }


    if ((strcmp(command, "money") == 0) || (strcmp(command, "profit") == 0)) {
        PrintEconomy();
        return;
    }

    if (strcmp(command, "pot") == 0) {
        const MAINTENANCE_LEVEL_INFO *info;
        char out_str[180];
        info = GetMaintenanceInfo(SelectedMaintLevel);
        sprintf(out_str, "pot_adc=%lu selected=%s cost=%lu health_gain=%lu downtime=%lu sec\r\n", (unsigned long)PotAdcRaw, info->Name, (unsigned long)info->Cost, (unsigned long)info->HealthGain, (unsigned long)info->DowntimeSec);
        UsartPrint(out_str);
        return;
    }

    if ((strcmp(command, "maint") == 0) || (strcmp(command, "maintenance") == 0)) {
        ApplySelectedMaintenance();
        return;
    }

    if (strcmp(command, "clear") == 0) {
        if (EmergencyActive == DEF_FALSE) {
            UsartPrint("No emergency is active.\r\n");
            return;
        }
        if (GPIO_ReadInputDataBit(GPIOC, ESTOP_BUTTON_PIN) == Bit_SET) {
            UsartPrint("Cannot clear: release physical E-STOP first.\r\n");
            return;
        }
        if (CurrentPersonState == PERSON_NEAR) {
            UsartPrint("Cannot clear: person still detected near the sensor.\r\n");
            return;
        }
        if (MachineHealth == MACHINE_HEALTH_MIN) {
            UsartPrint("Cannot clear: MachineHealth is 0. Use 'maint' first.\r\n");
            return;
        }

        CPU_CRITICAL_ENTER();
        EmergencyActive          = DEF_FALSE;
        SensorEmergencyRequest   = DEF_FALSE;
        SystemRun                = DEF_FALSE;
        ConveyorAState           = SYS_STATE_NORMAL;
        ConveyorBState           = SYS_STATE_NORMAL;
        OverallState             = SYS_STATE_NORMAL;
        ConveyorADelayMs         = DELAY_NORMAL_MS;
        ConveyorBDelayMs         = DELAY_NORMAL_MS;
        DoorHoldClosedUntilStart = DEF_TRUE;
        CPU_CRITICAL_EXIT();

        BuzzerOff();
        OSSemPost(&RgbUpdateSem, OS_OPT_POST_1, &err);
        UsartPrint("Emergency cleared. Type 'start' to physically reopen gate and restart.\r\n");
        return;
    }

    if ((strcmp(command, "log on") == 0) || (strcmp(command, "log off") == 0) || (strcmp(command, "log") == 0)) {
        SensorLogEnabled = DEF_FALSE;
        UsartPrint("Periodic logs disabled.\r\n");
        return;
    }

    if (strcmp(command, "help") == 0) {
        UsartPrint("Commands:\r\n"
                   "  start         - physically open servo (600ms), then start conveyors\r\n"
                   "  stop          - physically close servo to 90 deg (600ms), then stop\r\n"
                   "  status        - print current status\r\n"
                   "  count         - print product counts\r\n"
                   "  ultra         - print ultrasonic distance/person state\r\n"
                   "  money         - print profit / cost / net\r\n"
                   "  pot           - print potentiometer maintenance level\r\n"
                   "  maint         - apply maintenance if health <= 70\r\n"
                   "  clear         - clear latched emergency\r\n"
                   "  help          - this list\r\n");
        return;
    }

    UsartPrint("Unknown command. Type 'help'.\r\n");
}

static void PrintStatus(void)
{
    SENSOR_DATA sample;
    const MAINTENANCE_LEVEL_INFO *info;
    char message[640];
    CPU_SR_ALLOC();

    UpdateFactoryEconomy();

    CPU_CRITICAL_ENTER();
    sample = LatestSensorData;
    info   = GetMaintenanceInfo(SelectedMaintLevel);

    snprintf(message, sizeof(message),
             "run=%u emergency=%u maintenance=%u door_hold=%u servo_lock=%u\r\n"
             "x=%d y=%d z=%d\r\n"
             "A[state=%s vib=%lu delay=%lu]\r\n"
             "B[state=%s vib=%lu delay=%lu]\r\n"
             "person=%s dist=%lu cm\r\n"
             "door=%s (0deg=open 45deg=warn 90deg=locked/stop)\r\n"
             "health=%lu/%lu maintenance_allowed=%s\r\n"
             "pot_adc=%lu selected=%s\r\n"
             "runtime=%lu sec products=%lu profit=%ld cost=%ld net=%ld\r\n",
             (unsigned int)SystemRun, (unsigned int)EmergencyActive, (unsigned int)MaintenanceInProgress, (unsigned int)DoorHoldClosedUntilStart, (unsigned int)ServoHardLock, sample.X, sample.Y, sample.Z, StateToString(ConveyorAState), (unsigned long)sample.VibA, (unsigned long)ConveyorADelayMs, StateToString(ConveyorBState), (unsigned long)sample.VibB, (unsigned long)ConveyorBDelayMs, PersonStateToString(CurrentPersonState), (unsigned long)LastDistanceCm, DoorStateToString(CurrentDoorState), (unsigned long)MachineHealth, (unsigned long)MACHINE_HEALTH_MAX, (MachineHealth <= MAINT_ALLOWED_HEALTH) ? "YES" : "NO", (unsigned long)PotAdcRaw, info->Name, (unsigned long)FactoryRunTimeSec, (unsigned long)(ProductCountA + ProductCountB), (long)FactoryProfit, (long)TotalMaintenanceCost, (long)FactoryNetProfit);
    CPU_CRITICAL_EXIT();

    UsartPrint(message);
}

static void UsartPutCharRaw(char character)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART3, (uint16_t)character);
}

static void UsartPrint(const char *text)
{
    OS_ERR err;
    CPU_TS ts;
    OSMutexPend(&UartMutex, 0u, OS_OPT_PEND_BLOCKING, &ts, &err);
    if (err != OS_ERR_NONE) { return; }
    while (*text != '\0') { UsartPutCharRaw(*text++); }
    OSMutexPost(&UartMutex, OS_OPT_POST_NONE, &err);
}

/* =========================================================
 * State helpers
 * =========================================================*/

static SYSTEM_STATE GetOverallState(void) { return OverallState; }

static uint32_t StateToDelay(SYSTEM_STATE state)
{
    switch (state) {
        case SYS_STATE_NORMAL:   return DELAY_NORMAL_MS;
        case SYS_STATE_WARNING:  return DELAY_WARNING_MS;
        case SYS_STATE_RECOVERY: return DELAY_RECOVERY_MS;
        case SYS_STATE_DANGER:   return DELAY_DANGER_MS;
        default:                 return DELAY_RECOVERY_MS;
    }
}

static const char *StateToString(SYSTEM_STATE state)
{
    switch (state) {
        case SYS_STATE_NORMAL:    return "NORMAL";
        case SYS_STATE_WARNING:   return "WARNING";
        case SYS_STATE_RECOVERY:  return "RECOVERY";
        case SYS_STATE_DANGER:    return "DANGER";
        case SYS_STATE_EMERGENCY: return "EMERGENCY";
        default:                  return "UNKNOWN";
    }
}

static const char *PersonStateToString(PERSON_STATE ps)
{
    switch (ps) {
        case PERSON_NONE: return "NONE";
        case PERSON_WARN: return "WARN";
        case PERSON_NEAR: return "NEAR";
        default:          return "UNKNOWN";
    }
}

static const char *DoorStateToString(DOOR_STATE ds)
{
    switch (ds) {
        case DOOR_OPEN:    return "OPEN(0deg)";
        case DOOR_CLOSING: return "CLOSING";
        case DOOR_HALF:    return "HALF(45deg)";
        case DOOR_CLOSED:  return "CLOSED(90deg)";
        case DOOR_LOCKED:  return "LOCKED(90deg)";
        default:           return "UNKNOWN";
    }
}

/* =========================================================
 * ADXL345 / I2C helpers
 * =========================================================*/

static CPU_BOOLEAN I2CWaitEvent(I2C_TypeDef *i2c, uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(i2c, event) == ERROR) {
        if (timeout-- == 0u) { return DEF_FALSE; }
    }
    return DEF_TRUE;
}

static CPU_BOOLEAN I2CWaitWhileFlag(I2C_TypeDef *i2c, uint32_t flag, FlagStatus value)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(i2c, flag) == value) {
        if (timeout-- == 0u) { return DEF_FALSE; }
    }
    return DEF_TRUE;
}

static void I2CAbort(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

static CPU_BOOLEAN ADXLReadReg(uint8_t reg, uint8_t *value)
{
    if (I2CWaitWhileFlag(I2C1, I2C_FLAG_BUSY, SET) == DEF_FALSE) goto fail;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == DEF_FALSE) goto fail;
    I2C_Send7bitAddress(I2C1, ADXL345_ADDR, I2C_Direction_Transmitter);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == DEF_FALSE) goto fail;
    I2C_SendData(I2C1, reg);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == DEF_FALSE) goto fail;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == DEF_FALSE) goto fail;
    I2C_Send7bitAddress(I2C1, ADXL345_ADDR, I2C_Direction_Receiver);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == DEF_FALSE) goto fail;
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == DEF_FALSE) goto fail;
    *value = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return DEF_TRUE;
fail:
    I2CAbort();
    return DEF_FALSE;
}

static CPU_BOOLEAN ADXLWriteReg(uint8_t reg, uint8_t value)
{
    if (I2CWaitWhileFlag(I2C1, I2C_FLAG_BUSY, SET) == DEF_FALSE) goto fail;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == DEF_FALSE) goto fail;
    I2C_Send7bitAddress(I2C1, ADXL345_ADDR, I2C_Direction_Transmitter);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == DEF_FALSE) goto fail;
    I2C_SendData(I2C1, reg);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == DEF_FALSE) goto fail;
    I2C_SendData(I2C1, value);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == DEF_FALSE) goto fail;
    I2C_GenerateSTOP(I2C1, ENABLE);
    return DEF_TRUE;
fail:
    I2CAbort();
    return DEF_FALSE;
}

static CPU_BOOLEAN ADXLReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    uint32_t index;
    if (I2CWaitWhileFlag(I2C1, I2C_FLAG_BUSY, SET) == DEF_FALSE) goto fail;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == DEF_FALSE) goto fail;
    I2C_Send7bitAddress(I2C1, ADXL345_ADDR, I2C_Direction_Transmitter);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == DEF_FALSE) goto fail;
    I2C_SendData(I2C1, ADXL_REG_DATAX0);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == DEF_FALSE) goto fail;
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == DEF_FALSE) goto fail;
    I2C_Send7bitAddress(I2C1, ADXL345_ADDR, I2C_Direction_Receiver);
    if (I2CWaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == DEF_FALSE) goto fail;
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    for (index = 0u; index < 6u; index++) {
        if (index == 5u) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        if (I2CWaitWhileFlag(I2C1, I2C_FLAG_RXNE, RESET) == DEF_FALSE) goto fail;
        data[index] = I2C_ReceiveData(I2C1);
    }
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    *x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    *y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    *z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    return DEF_TRUE;
fail:
    I2CAbort();
    return DEF_FALSE;
}

/* =========================================================
 * Hardware setup
 * =========================================================*/

static void SetupGpio(void)
{
    GPIO_InitTypeDef gpio;

    DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin   = CONVEYOR_ALL_MASK;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, CONVEYOR_ALL_MASK);

    gpio.GPIO_Pin = RGB_ALL_MASK | BUZZER_PIN;
    GPIO_Init(GPIOC, &gpio);
    GPIO_ResetBits(GPIOC, RGB_ALL_MASK | BUZZER_PIN);

    gpio.GPIO_Pin  = START_BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin  = MAINT_BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin  = ESTOP_BUTTON_PIN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin   = ULTRA_TRIG_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, ULTRA_TRIG_PIN);

    gpio.GPIO_Pin  = ULTRA_ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);
}

static void SetupServoTim5(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       oc;
    RCC_ClocksTypeDef       clocks;
    uint32_t                tim5_clock_hz;
    uint32_t                prescaler_val;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

    gpio.GPIO_Pin   = SERVO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    RCC_GetClocksFreq(&clocks);
    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) {
        tim5_clock_hz = clocks.PCLK1_Frequency;
    } else {
        tim5_clock_hz = clocks.PCLK1_Frequency * 2u;
    }
    prescaler_val = (tim5_clock_hz / 1000000u) - 1u;

    tim.TIM_Period        = SERVO_PWM_PERIOD_US - 1u;
    tim.TIM_Prescaler     = (uint16_t)prescaler_val;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = SERVO_PULSE_OPEN_US;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM5, &oc);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

static void SetupAdc1Potentiometer(void)
{
    GPIO_InitTypeDef       gpio;
    ADC_InitTypeDef        adc;
    ADC_CommonInitTypeDef  adc_common;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);

    gpio.GPIO_Pin   = POT_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POT_GPIO, &gpio);

    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode             = ADC_Mode_Independent;
    adc_common.ADC_Prescaler        = ADC_Prescaler_Div4;
    adc_common.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    ADC_StructInit(&adc);
    adc.ADC_Resolution           = ADC_Resolution_12b;
    adc.ADC_ScanConvMode         = DISABLE;
    adc.ADC_ContinuousConvMode   = DISABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign            = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion      = 1u;
    ADC_Init(POT_ADC, &adc);

    ADC_RegularChannelConfig(POT_ADC, POT_ADC_CHANNEL, 1u, ADC_SampleTime_144Cycles);
    ADC_Cmd(POT_ADC, ENABLE);
}

static void SetupEmergencyExti(void)
{
    EXTI_InitTypeDef exti;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    EXTI_ClearITPendingBit(EXTI_Line1);
    exti.EXTI_Line    = EXTI_Line1;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    BSP_IntVectSet(BSP_INT_ID_EXTI1, AppEmergencyISR);
    BSP_IntPrioSet(BSP_INT_ID_EXTI1, 5u);
    BSP_IntEn(BSP_INT_ID_EXTI1);
}

static void SetupI2C1(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00u;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = 100000u;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

static void SetupUsart3(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    usart.USART_BaudRate            = 115200u;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);
    USART_Cmd(USART3, ENABLE);
}

static void AllConveyorsOff(void)
{
    GPIO_ResetBits(GPIOB, CONVEYOR_ALL_MASK);
}
