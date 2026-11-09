#include "bq7690x_power_modes.h"

/* Command the device to enter DEEPSLEEP mode (automatically sent twice within 4s) */
bq_status_t bq7690x_deep_sleep(bq7690x_t *dev)
{
    if (!dev) return BQ_ERR_PARAM;

    bq_status_t st = bq_command_subcmd(dev, CMD_DEEPSLEEP);
    if (st != BQ_OK) return st;

    /* Second send must follow within 4s */
    dev->delay_ms(100);

    return bq_command_subcmd(dev, CMD_DEEPSLEEP);
}

/* Command the device to exit DEEPSLEEP mode */
bq_status_t bq7690x_exit_deep_sleep(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_EXIT_DEEPSLEEP);
}

/* Command the device to enter SHUTDOWN mode (automatically sent twice) */
bq_status_t bq7690x_shutdown(bq7690x_t *dev)
{
    if (!dev) return BQ_ERR_PARAM;

    bq_status_t st = bq_command_subcmd(dev, CMD_SHUTDOWN);
    if (st != BQ_OK) return st;

    dev->delay_ms(1);
    return bq_command_subcmd(dev, CMD_SHUTDOWN);
}

/* Allow the device to enter SLEEP mode */
bq_status_t bq7690x_sleep_enable(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_SLEEP_ENABLE);
}

/* Disallow the device from entering SLEEP mode */
bq_status_t bq7690x_sleep_disable(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_SLEEP_DISABLE);
}
