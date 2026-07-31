#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * MVP parameters. For the real brew, change only BREW_TIME_MS to:
 * 3 * 60 * 1000.
 */
static const int64_t BREW_TIME_MS = 20 * 1000;
static const int64_t FILL_CONFIRM_MS = 1500;
static const int64_t EMPTY_CONFIRM_MS = 3000;
static const TickType_t LOOP_PERIOD = pdMS_TO_TICKS(20);

static const char *TAG = "diy_cup";

/* Safe GPIO selection for a classic 38-pin ESP32 DevKit/WROOM-32 board. */
#define REED_GPIO GPIO_NUM_27
#define SERVO_GPIO GPIO_NUM_18
#define PIEZO_GPIO GPIO_NUM_25
#define BLUE_LED_GPIO GPIO_NUM_26
#define GREEN_LED_GPIO GPIO_NUM_33

typedef enum {
    CUP_STATE_EMPTY,
    CUP_STATE_BREWING,
    CUP_STATE_READY,
} cup_state_t;

typedef struct {
    cup_state_t state;
    int64_t state_entered_at_ms;

    /* Start time of the currently checked stable sensor condition. */
    bool condition_timer_running;
    int64_t condition_started_at_ms;
} cup_controller_t;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool elapsed(int64_t now, int64_t started_at, int64_t duration_ms)
{
    return (now - started_at) >= duration_ms;
}

static bool read_reed_active(void)
{
    /* The reed switch connects GPIO27 to GND; the internal pull-up is enabled. */
    return gpio_get_level(REED_GPIO) == 0;
}

static void set_blue_led(bool enabled)
{
    gpio_set_level(BLUE_LED_GPIO, enabled ? 1 : 0);
}

static void set_green_led(bool enabled)
{
    gpio_set_level(GREEN_LED_GPIO, enabled ? 1 : 0);
}

static void move_teabag_down(void)
{
    /* TODO: set SG90 PWM to SERVO_DOWN_ANGLE. */
}

static void move_teabag_up(void)
{
    /* TODO: set SG90 PWM to SERVO_UP_ANGLE. */
}

static void play_ready_signal(void)
{
    /* TODO: start a short non-blocking piezo signal. */
}

static void hardware_init(void)
{
    const gpio_config_t reed_config = {
        .pin_bit_mask = 1ULL << REED_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    const gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << BLUE_LED_GPIO) |
                        (1ULL << GREEN_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&reed_config));
    ESP_ERROR_CHECK(gpio_config(&led_config));

    set_blue_led(false);
    set_green_led(false);

    ESP_LOGI(TAG,
             "GPIO: reed=%d, servo=%d, piezo=%d, blue=%d, green=%d",
             REED_GPIO, SERVO_GPIO, PIEZO_GPIO, BLUE_LED_GPIO,
             GREEN_LED_GPIO);
}

static void reset_condition_timer(cup_controller_t *controller)
{
    controller->condition_timer_running = false;
    controller->condition_started_at_ms = 0;
}

static bool condition_is_stable_for(cup_controller_t *controller,
                                    bool condition,
                                    int64_t now,
                                    int64_t confirmation_ms)
{
    if (!condition) {
        reset_condition_timer(controller);
        return false;
    }

    if (!controller->condition_timer_running) {
        controller->condition_timer_running = true;
        controller->condition_started_at_ms = now;
        return false;
    }

    return elapsed(now, controller->condition_started_at_ms, confirmation_ms);
}

static const char *state_name(cup_state_t state)
{
    switch (state) {
    case CUP_STATE_EMPTY:
        return "EMPTY";
    case CUP_STATE_BREWING:
        return "BREWING";
    case CUP_STATE_READY:
        return "READY";
    default:
        return "UNKNOWN";
    }
}

static void enter_state(cup_controller_t *controller,
                        cup_state_t next_state,
                        int64_t now)
{
    const cup_state_t previous_state = controller->state;

    controller->state = next_state;
    controller->state_entered_at_ms = now;
    reset_condition_timer(controller);

    ESP_LOGI(TAG, "%s -> %s", state_name(previous_state),
             state_name(next_state));

    switch (next_state) {
    case CUP_STATE_EMPTY:
        set_blue_led(false);
        set_green_led(false);
        move_teabag_down();
        break;

    case CUP_STATE_BREWING:
        set_blue_led(true);
        set_green_led(false);
        move_teabag_down();
        break;

    case CUP_STATE_READY:
        set_blue_led(false);
        set_green_led(true);
        move_teabag_up();
        play_ready_signal();
        break;
    }
}

static void controller_init(cup_controller_t *controller, int64_t now)
{
    controller->state = CUP_STATE_EMPTY;
    controller->state_entered_at_ms = now;
    reset_condition_timer(controller);

    set_blue_led(false);
    set_green_led(false);
    move_teabag_down();

    ESP_LOGI(TAG, "Initial state: EMPTY");
}

static void controller_update(cup_controller_t *controller,
                              bool reed_active,
                              int64_t now)
{
    switch (controller->state) {
    case CUP_STATE_EMPTY:
        /* A splash or contact bounce resets this confirmation interval. */
        if (condition_is_stable_for(controller, reed_active, now,
                                    FILL_CONFIRM_MS)) {
            enter_state(controller, CUP_STATE_BREWING, now);
        }
        break;

    case CUP_STATE_BREWING:
        /* Sensor changes do not restart or extend an active brew. */
        if (elapsed(now, controller->state_entered_at_ms, BREW_TIME_MS)) {
            enter_state(controller, CUP_STATE_READY, now);
        }
        break;

    case CUP_STATE_READY:
        /* A new brew is impossible until empty is confirmed continuously. */
        if (condition_is_stable_for(controller, !reed_active, now,
                                    EMPTY_CONFIRM_MS)) {
            enter_state(controller, CUP_STATE_EMPTY, now);
        }
        break;
    }
}

void app_main(void)
{
    cup_controller_t controller;

    hardware_init();
    controller_init(&controller, now_ms());

    while (true) {
        const int64_t current_time_ms = now_ms();
        const bool reed_active = read_reed_active();

        controller_update(&controller, reed_active, current_time_ms);
        vTaskDelay(LOOP_PERIOD);
    }
}
