#include "chassis_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "board_config.h"
#include "chassis_app.h"
#include "stm32f4xx_hal.h"

#define CONTROL_TASK_STACK_WORDS       1024U
#define SENSOR_TASK_STACK_WORDS         384U
#define TELEMETRY_TASK_STACK_WORDS      512U
#define HOUSEKEEPING_TASK_STACK_WORDS   192U

#define CONTROL_TASK_PRIORITY          (tskIDLE_PRIORITY + 5U)
#define SENSOR_TASK_PRIORITY           (tskIDLE_PRIORITY + 4U)
#define TELEMETRY_TASK_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define HOUSEKEEPING_TASK_PRIORITY     (tskIDLE_PRIORITY + 1U)

static StaticTask_t control_task_tcb;
static StaticTask_t sensor_task_tcb;
static StaticTask_t telemetry_task_tcb;
static StaticTask_t housekeeping_task_tcb;
static StackType_t control_task_stack[CONTROL_TASK_STACK_WORDS];
static StackType_t sensor_task_stack[SENSOR_TASK_STACK_WORDS];
static StackType_t telemetry_task_stack[TELEMETRY_TASK_STACK_WORDS];
static StackType_t housekeeping_task_stack[HOUSEKEEPING_TASK_STACK_WORDS];

static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

static void control_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;) {
        chassis_app_control_process(HAL_GetTick());
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_CONTROL_SERVICE_PERIOD_MS));
    }
}

static void sensor_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;) {
        chassis_app_sensor_process(HAL_GetTick());
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_SENSOR_SERVICE_PERIOD_MS));
    }
}

static void telemetry_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;) {
        chassis_app_telemetry_process(HAL_GetTick());
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_TELEMETRY_SERVICE_PERIOD_MS));
    }
}

static void housekeeping_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;) {
        chassis_app_housekeeping_process(HAL_GetTick());
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_HOUSEKEEPING_PERIOD_MS));
    }
}

void chassis_tasks_start(void)
{
    TaskHandle_t control_handle;
    TaskHandle_t sensor_handle;
    TaskHandle_t telemetry_handle;
    TaskHandle_t housekeeping_handle;

    control_handle = xTaskCreateStatic(control_task,
                                       "chassis_ctrl",
                                       CONTROL_TASK_STACK_WORDS,
                                       NULL,
                                       CONTROL_TASK_PRIORITY,
                                       control_task_stack,
                                       &control_task_tcb);
    sensor_handle = xTaskCreateStatic(sensor_task,
                                      "sensors",
                                      SENSOR_TASK_STACK_WORDS,
                                      NULL,
                                      SENSOR_TASK_PRIORITY,
                                      sensor_task_stack,
                                      &sensor_task_tcb);
    telemetry_handle = xTaskCreateStatic(telemetry_task,
                                         "telemetry",
                                         TELEMETRY_TASK_STACK_WORDS,
                                         NULL,
                                         TELEMETRY_TASK_PRIORITY,
                                         telemetry_task_stack,
                                         &telemetry_task_tcb);
    housekeeping_handle = xTaskCreateStatic(housekeeping_task,
                                            "housekeeping",
                                            HOUSEKEEPING_TASK_STACK_WORDS,
                                            NULL,
                                            HOUSEKEEPING_TASK_PRIORITY,
                                            housekeeping_task_stack,
                                            &housekeeping_task_tcb);
    if (control_handle == NULL || sensor_handle == NULL
        || telemetry_handle == NULL || housekeeping_handle == NULL) {
        board_fatal_error();
    }

    vTaskStartScheduler();
    board_fatal_error();
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                   StackType_t **stack_buffer,
                                   uint32_t *stack_size)
{
    *task_buffer = &idle_task_tcb;
    *stack_buffer = idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationIdleHook(void)
{
    __WFI();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    board_fatal_error();
}
