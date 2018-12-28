#pragma once

#include <stdint.h>

static const uint8_t EXCEPTION_I2C_START           = 1U;
static const uint8_t EXCEPTION_I2C_WRITE           = 2U;
static const uint8_t EXCEPTION_I2C_REP_START       = 3U;
static const uint8_t EXCEPTION_NOT_IN_NORMAL_MODE0 = 4U;

typedef struct accel
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;

typedef struct d7s_earthquake_related_data
{
    uint16_t si;
    uint16_t pga;
} d7s_earthquake_related_data_t;

struct d7s_latest_data
{
    accel_t  accel;
    int16_t  ave;
    uint16_t si;
    uint16_t pga;
};

typedef struct d7s_latest_data d7s_latest_data_t;
typedef struct d7s_latest_data d7s_si_ranked_data_t;

typedef struct d7s_initial_installation_data
{
    accel_t accel;
    int16_t ave;
    accel_t max;
    accel_t min;
    uint8_t axis;
} d7s_initial_installation_data_t;

typedef struct d7s_latest_offset_data
{
    accel_t accel;
    int16_t ave;
    accel_t max;
    accel_t min;
    uint8_t state;
} d7s_latest_offset_data_t;

typedef struct d7s_self_diagnostic_data
{
    int16_t before_x;
    int16_t after_x;
    int16_t before_y;
    int16_t after_y;
    int16_t before_z;
    int16_t after_z;
    int16_t ave;
    uint8_t error;
} d7s_self_diagnostic_data_t;

void d7s_init(void);

uint8_t d7s_query_state(void);
uint8_t d7s_query_axis_state(void);

uint8_t d7s_query_shutoff_signal_on_in_earthquake(void);
uint8_t d7s_query_shutoff_signal_on_in_collapse(void);
uint8_t d7s_query_self_diagnostic_error(void);
uint8_t d7s_query_acquire_offset_error(void);

uint8_t d7s_query_mode(void);
uint8_t d7s_query_earthquake_shutoff_judgement_threshold(void);
uint8_t d7s_query_si_value_calculation_axes(void);

uint8_t d7s_query_clear_initial_installation_data_memory(void);
uint8_t d7s_query_clear_latest_offset_data_memory(void);
uint8_t d7s_query_clear_self_diagnostic_data_memory(void);
uint8_t d7s_query_clear_earthquake_data_memory(void);

/* During an earthquake, one can acquire the SI value and PGA currently being
 * calculated by executing the following query function: */
d7s_earthquake_related_data_t d7s_query_earthquake_related_data(void);

/* After the earthquake ends, one can read the data for the past five
 * earthquakes by executing the following query functions (Latest Data 1
 * (register addresses 0x3000 to 0x300B) always holds the latest data): */
d7s_latest_data_t d7s_query_latest_data_1(void);
d7s_latest_data_t d7s_query_latest_data_2(void);
d7s_latest_data_t d7s_query_latest_data_3(void);
d7s_latest_data_t d7s_query_latest_data_4(void);
d7s_latest_data_t d7s_query_latest_data_5(void);

/* After the earthquake ends, one can read the data for five earthquakes
 * with the largest SI values, out of all earthquakes that occurred in the past,
 * by executing the following query functions (SI Ranked Data 1
 * (register addresses 0x3500 to 0x350B) always holds the largest SI value): */
d7s_si_ranked_data_t d7s_query_si_ranked_data_1(void);
d7s_si_ranked_data_t d7s_query_si_ranked_data_2(void);
d7s_si_ranked_data_t d7s_query_si_ranked_data_3(void);
d7s_si_ranked_data_t d7s_query_si_ranked_data_4(void);
d7s_si_ranked_data_t d7s_query_si_ranked_data_5(void);

d7s_initial_installation_data_t d7s_query_initial_installation_data(void);

d7s_latest_offset_data_t d7s_query_latest_offset_data(void);

d7s_self_diagnostic_data_t d7s_query_self_diagnostic_data(void);

// Switching to a mode is only possible if the current mode is the normal mode!
void d7s_set_mode_to_initial_installation_mode(void);
void d7s_set_mode_to_offset_acquisition_mode(void);
void d7s_set_mode_to_self_diagnostic_mode(void);

/* The default is threshold level is high, and the shutoff signal
 * will be output if an earthquake occurs with a seismic intensity equivalent
 * to 5 Upper or higher on the JMA Seismic Intensity Scale. */
void d7s_set_earthquake_shutoff_judgement_threshold_to_high(void);
void d7s_set_earthquake_shutoff_judgement_threshold_to_low(void);

void d7s_clear_initial_installation_data_memory(void);
void d7s_clear_self_diagnostic_data_memory(void);
void d7s_clear_latest_offset_data_memory(void);
void d7s_clear_earthquake_data_memory(void);

void d7s_set_ctrl_axis_to_switch_axes_at_installation(void);
void d7s_set_ctrl_axis_to_auto_switch_axes(void);
void d7s_set_ctrl_axis_to_xy_axes(void);
void d7s_set_ctrl_axis_to_xz_axes(void);
void d7s_set_ctrl_axis_to_yz_axes(void);
