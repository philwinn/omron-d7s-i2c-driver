#include "omron_d7s.h"
#include "i2cmaster.h"
#include "CException.h"

// Omron D7S I2C 8 bit slave device address
static const uint8_t d7s_addr = 0xaa;

typedef struct reg
{
    const uint16_t adr;
    const uint8_t  size;
} reg_t;

static const reg_t reg_state                     = {0x1000U, 1U};
static const reg_t reg_axis_state                = {0x1001U, 1U};
static const reg_t reg_event                     = {0x1002U, 1U};
static const reg_t reg_mode                      = {0x1003U, 1U};
static const reg_t reg_ctrl                      = {0x1004U, 1U};
static const reg_t reg_clear_command             = {0x1005U, 1U};

static const reg_t reg_earthquake_related_data   = {0x2000U, 4U};

static const reg_t reg_latest_data_1             = {0x3000U, 12U};
static const reg_t reg_latest_data_2             = {0x3100U, 12U};
static const reg_t reg_latest_data_3             = {0x3200U, 12U};
static const reg_t reg_latest_data_4             = {0x3300U, 12U};
static const reg_t reg_latest_data_5             = {0x3400U, 12U};

static const reg_t reg_si_ranked_data_1          = {0x3500U, 12U};
static const reg_t reg_si_ranked_data_2          = {0x3600U, 12U};
static const reg_t reg_si_ranked_data_3          = {0x3700U, 12U};
static const reg_t reg_si_ranked_data_4          = {0x3800U, 12U};
static const reg_t reg_si_ranked_data_5          = {0x3900U, 12U};

static const reg_t reg_initial_installation_data = {0x4000U, 21U};
static const reg_t reg_latest_offset_data        = {0x4100U, 21U};
static const reg_t reg_self_diagnostic_data      = {0x4200U, 15U};

// D7S modes
static const uint8_t mode_normal               = 0x01U;
static const uint8_t mode_initial_installation = 0x02U;
static const uint8_t mode_offste_acquisition   = 0x03U;
static const uint8_t mode_self_diagnostic      = 0x04U;

// D7S clear commands
static const uint8_t clear_earthquake_data_memory           = 0x01U;
static const uint8_t clear_self_diagnostic_data_memory      = 0x02U;
static const uint8_t clear_latest_offset_data_memory        = 0x04U;
static const uint8_t clear_initial_installation_data_memory = 0x08U;

// D7S CTRL axis arguments
static const uint8_t ctrl_axis_switch_axes_at_installation = 0x04U;
static const uint8_t ctrl_axis_auto_switch_axes            = 0x03U;
static const uint8_t ctrl_axis_xy_axes                     = 0x02U;
static const uint8_t ctrl_axis_xz_axes                     = 0x01U;
static const uint8_t ctrl_axis_yz_axes                     = 0x00U;

static const uint8_t threshold_high = 0x01U;
static const uint8_t threshold_low  = 0x00U;

typedef struct bitmask
{
    const uint8_t mask;
    const uint8_t shift;
} bitmask_t;

static const bitmask_t bitmask_0   = {0b00000001U, 0U};
static const bitmask_t bitmask_1   = {0b00000010U, 1U};
static const bitmask_t bitmask_2   = {0b00000100U, 2U};
static const bitmask_t bitmask_3   = {0b00001000U, 3U};
static const bitmask_t bitmask_10  = {0b00000011U, 0U};
static const bitmask_t bitmask_210 = {0b00000111U, 0U};
static const bitmask_t bitmask_654 = {0b01110000U, 4U};

static int16_t
merge_signed(const uint8_t high,
             const uint8_t low)
{
    return ((int16_t) ((high << 8) | (low & 0xff)));
}

static uint16_t
merge_unsigned(const uint8_t high,
               const uint8_t low)
{
    return ((uint16_t) ((high << 8) | (low & 0xff)));
}

static uint8_t
low_byte(const uint16_t reg)
{
        uint8_t byte = (reg & 0xff);
        return byte;
}

static uint8_t
high_byte(const uint16_t reg)
{
        uint8_t byte = ((reg >> 8) & 0xff);
        return byte;
}

/* The function d7s_read_bytes reads bytes_to_read many bytes via I2C,
 * starting from register reg and writes the read bytes to the memory
 * the res pointer points to. */
static uint8_t
d7s_read_bytes(uint8_t bytes_to_read,
               const uint16_t reg,
               uint8_t * const res)
{
    if (bytes_to_read > 0)
    {
        i2c_start_wait(d7s_addr + I2C_WRITE);
        if (i2c_write(high_byte(reg)) || i2c_write(low_byte(reg)))
        {
            Throw(EXCEPTION_I2C_WRITE);
        }
        if (i2c_rep_start(d7s_addr + I2C_READ))
        {
            Throw(EXCEPTION_I2C_REP_START);
        }
        --bytes_to_read;
        for (uint8_t pos = 0; pos < bytes_to_read; pos++)
        {
            res[pos] = i2c_readAck();
        }
        res[bytes_to_read] = i2c_readNak();
        i2c_stop();
    }
}

static void
d7s_write_byte(const uint16_t reg,
               const uint8_t val)
{
    if (i2c_start(d7s_addr + I2C_WRITE))
    {
        Throw(EXCEPTION_I2C_START);
    }
    if (i2c_write(high_byte(reg)) || i2c_write(low_byte(reg)) || i2c_write(val))
    {
        Throw(EXCEPTION_I2C_WRITE);
    }
    i2c_stop();
}

void
d7s_init()
{
    i2c_init();
}

static uint8_t
d7s_query(const reg_t reg,
          const bitmask_t bm)
{
    uint8_t res = 0;
    d7s_read_bytes(reg.size, reg.adr, &res);
    res &= bm.mask;
    res = (res >> bm.shift);
    return res;
}

uint8_t
d7s_query_state()
{
    return d7s_query(reg_state, bitmask_210);
}

uint8_t
d7s_query_axis_state()
{
    return d7s_query(reg_axis_state, bitmask_10);
}

uint8_t
d7s_query_shutoff_signal_on_in_earthquake()
{
    return d7s_query(reg_event, bitmask_0);
}

uint8_t
d7s_query_shutoff_signal_on_in_collapse()
{
    return d7s_query(reg_event, bitmask_1);
}

uint8_t
d7s_query_self_diagnostic_error()
{
    return d7s_query(reg_event, bitmask_2);
}

uint8_t
d7s_query_acquire_offset_error()
{
    return d7s_query(reg_event, bitmask_3);
}

uint8_t
d7s_query_mode()
{
    return d7s_query(reg_mode, bitmask_210);
}

uint8_t
d7s_query_earthquake_shutoff_judgement_threshold()
{
    return d7s_query(reg_ctrl, bitmask_3);
}

uint8_t
d7s_query_si_value_calculation_axes()
{
    return d7s_query(reg_ctrl, bitmask_654);
}

uint8_t
d7s_query_clear_initial_installation_data_memory(void)
{
    return d7s_query(reg_clear_command, bitmask_3);
}

uint8_t
d7s_query_clear_latest_offset_data_memory(void)
{
    return d7s_query(reg_clear_command, bitmask_2);
}

uint8_t
d7s_query_clear_self_diagnostic_data_memory(void)
{
    return d7s_query(reg_clear_command, bitmask_1);
}

uint8_t
d7s_query_clear_earthquake_data_memory(void)
{
    return d7s_query(reg_clear_command, bitmask_0);
}

d7s_earthquake_related_data_t
d7s_query_earthquake_related_data(void)
{
    uint8_t buf[reg_earthquake_related_data.size];
    d7s_earthquake_related_data_t data;

    d7s_read_bytes(reg_earthquake_related_data.size,
                   reg_earthquake_related_data.adr,
                   buf);

    data.si  = merge_unsigned(buf[0], buf[1]);
    data.pga = merge_unsigned(buf[2], buf[3]);

    return data;
}

static d7s_latest_data_t
d7s_query_latest_data(const reg_t reg)
{
    uint8_t buf[reg.size];
    d7s_latest_data_t data;

    d7s_read_bytes(reg.size, reg.adr, buf);

    data.accel.x = merge_signed(buf[0], buf[1]);
    data.accel.y = merge_signed(buf[2], buf[3]);
    data.accel.z = merge_signed(buf[4], buf[5]);
    data.ave     = merge_signed(buf[6], buf[7]);
    data.si      = merge_unsigned(buf[8], buf[9]);
    data.pga     = merge_unsigned(buf[10], buf[11]);

    return data;
}

d7s_latest_data_t
d7s_query_latest_data_1(void)
{
    return d7s_query_latest_data(reg_latest_data_1);
}

d7s_latest_data_t
d7s_query_latest_data_2(void)
{
    return d7s_query_latest_data(reg_latest_data_2);
}

d7s_latest_data_t
d7s_query_latest_data_3(void)
{
    return d7s_query_latest_data(reg_latest_data_3);
}

d7s_latest_data_t
d7s_query_latest_data_4(void)
{
    return d7s_query_latest_data(reg_latest_data_4);
}

d7s_latest_data_t
d7s_query_latest_data_5(void)
{
    return d7s_query_latest_data(reg_latest_data_5);
}

d7s_si_ranked_data_t
d7s_query_si_ranked_data_1(void)
{
    return (d7s_si_ranked_data_t) d7s_query_latest_data(reg_si_ranked_data_1);
}

d7s_si_ranked_data_t
d7s_query_si_ranked_data_2(void)
{
    return (d7s_si_ranked_data_t) d7s_query_latest_data(reg_si_ranked_data_2);
}

d7s_si_ranked_data_t
d7s_query_si_ranked_data_3(void)
{
    return (d7s_si_ranked_data_t) d7s_query_latest_data(reg_si_ranked_data_3);
}

d7s_si_ranked_data_t
d7s_query_si_ranked_data_4(void)
{
    return (d7s_si_ranked_data_t) d7s_query_latest_data(reg_si_ranked_data_4);
}

d7s_si_ranked_data_t
d7s_query_si_ranked_data_5(void)
{
    return (d7s_si_ranked_data_t) d7s_query_latest_data(reg_si_ranked_data_5);
}

// Initial Installation Data (0x4000-0x4014)
d7s_initial_installation_data_t
d7s_query_initial_installation_data(void)
{
    uint8_t buf[reg_initial_installation_data.size];
    d7s_initial_installation_data_t data;

    d7s_read_bytes(reg_initial_installation_data.size,
                   reg_initial_installation_data.adr,
                   buf);

    data.accel.x = merge_signed(buf[0], buf[1]);
    data.accel.y = merge_signed(buf[2], buf[3]);
    data.accel.z = merge_signed(buf[4], buf[5]);
    data.ave     = merge_signed(buf[6], buf[7]);
    data.max.x   = merge_signed(buf[8], buf[9]);
    data.max.y   = merge_signed(buf[10], buf[11]);
    data.max.z   = merge_signed(buf[12], buf[13]);
    data.min.x   = merge_signed(buf[14], buf[15]);
    data.min.y   = merge_signed(buf[16], buf[17]);
    data.min.z   = merge_signed(buf[18], buf[19]);
    data.axis    = buf[20];

    return data;
}

// Latest Offset Data (0x4100-0x4114)
d7s_latest_offset_data_t
d7s_query_latest_offset_data(void)
{
    uint8_t buf[reg_latest_offset_data.size];
    d7s_latest_offset_data_t data;

    d7s_read_bytes(reg_latest_offset_data.size,
                   reg_latest_offset_data.adr,
                   buf);

    data.accel.x = merge_signed(buf[0], buf[1]);
    data.accel.y = merge_signed(buf[2], buf[3]);
    data.accel.z = merge_signed(buf[4], buf[5]);
    data.ave     = merge_signed(buf[6], buf[7]);
    data.max.x   = merge_signed(buf[8], buf[9]);
    data.max.y   = merge_signed(buf[10], buf[11]);
    data.max.z   = merge_signed(buf[12], buf[13]);
    data.min.x   = merge_signed(buf[14], buf[15]);
    data.min.y   = merge_signed(buf[16], buf[17]);
    data.min.z   = merge_signed(buf[18], buf[19]);
    data.state   = buf[20];

    return data;
}

// Self-Diagnostic Data (0x4200-0x420E)
d7s_self_diagnostic_data_t
d7s_query_self_diagnostic_data(void)
{
    uint8_t buf[reg_self_diagnostic_data.size];
    d7s_self_diagnostic_data_t data;

    d7s_read_bytes(reg_self_diagnostic_data.size,
                   reg_self_diagnostic_data.adr,
                   buf);

    data.before_x = merge_signed(buf[0], buf[1]);
    data.after_x  = merge_signed(buf[2], buf[3]);
    data.before_y = merge_signed(buf[4], buf[5]);
    data.after_y  = merge_signed(buf[6], buf[7]);
    data.before_z = merge_signed(buf[8], buf[9]);
    data.after_z  = merge_signed(buf[10], buf[11]);
    data.ave      = merge_signed(buf[12], buf[13]);
    data.error    = buf[14];

    return data;
}

/* Switching to any mode is only allowed from normal mode, thus
 * this function checks if the current mode is the normal mode.*/
static void
d7s_set_mode(const uint8_t mode)
{
    if (d7s_query_mode() == mode_normal)
    {
        d7s_write_byte(reg_mode.adr, mode);
    }
    else
    {
        Throw(EXCEPTION_NOT_IN_NORMAL_MODE);
    }
}

void
d7s_set_mode_to_initial_installation_mode(void)
{
    d7s_set_mode(mode_initial_installation);
}

void
d7s_set_mode_to_offset_acquisition_mode(void)
{
    d7s_set_mode(mode_offste_acquisition);
}

void
d7s_set_mode_to_self_diagnostic_mode(void)
{
    d7s_set_mode(mode_self_diagnostic);
}

static void
d7s_set_earthquake_shutoff_judgement_threshold(const uint8_t threshold)
{
    uint8_t ctrl = 0;
    d7s_read_bytes(reg_ctrl.size,
                   reg_ctrl.adr,
                   &ctrl);

    ctrl &= ((uint8_t) ~bitmask_3.mask);
    ctrl += (threshold << bitmask_3.shift);

    d7s_write_byte(reg_ctrl.adr, ctrl);
}

void
d7s_set_earthquake_shutoff_judgement_threshold_to_high(void)
{
    d7s_set_earthquake_shutoff_judgement_threshold(threshold_high);
}

void
d7s_set_earthquake_shutoff_judgement_threshold_to_low(void)
{
    d7s_set_earthquake_shutoff_judgement_threshold(threshold_low);
}

void
d7s_clear_initial_installation_data_memory(void)
{
    d7s_write_byte(reg_clear_command.adr,
                   clear_initial_installation_data_memory);
}

void
d7s_clear_self_diagnostic_data_memory(void)
{
    d7s_write_byte(reg_clear_command.adr,
                   clear_self_diagnostic_data_memory);
}

void
d7s_clear_latest_offset_data_memory(void)
{
    d7s_write_byte(reg_clear_command.adr,
                   clear_latest_offset_data_memory);
}

void
d7s_clear_earthquake_data_memory(void)
{
    d7s_write_byte(reg_clear_command.adr,
                   clear_earthquake_data_memory);
}

static void
d7s_set_ctrl_axis(const uint8_t val)
{
    uint8_t reg_ctrl_val = 0;
    d7s_read_bytes(reg_ctrl.size,
                   reg_ctrl.adr,
                   &reg_ctrl_val);

    reg_ctrl_val &= ((uint8_t) ~bitmask_654.mask);
    reg_ctrl_val += (val << bitmask_654.shift);

    d7s_write_byte(reg_ctrl.adr, reg_ctrl_val);
}

void
d7s_set_ctrl_axis_to_switch_axes_at_installation(void)
{
    d7s_set_ctrl_axis(ctrl_axis_switch_axes_at_installation);
}

void
d7s_set_ctrl_axis_to_auto_switch_axes(void)
{
    d7s_set_ctrl_axis(ctrl_axis_auto_switch_axes);
}

void
d7s_set_ctrl_axis_to_xy_axes(void)
{
    d7s_set_ctrl_axis(ctrl_axis_xy_axes);
}

void
d7s_set_ctrl_axis_to_xz_axes(void)
{
    d7s_set_ctrl_axis(ctrl_axis_xz_axes);
}

void
d7s_set_ctrl_axis_to_yz_axes(void)
{
    d7s_set_ctrl_axis(ctrl_axis_yz_axes);
}
