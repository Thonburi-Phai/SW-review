#include "bq7690x_system.h"

/* Reset the device */
bq_status_t bq7690x_reset(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_RESET);
}

/* Reset the accumulated charge and timer */
bq_status_t bq7690x_reset_pass_q(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_RESET_PASSQ);
}

/* Toggle the FET_EN bit */
bq_status_t bq7690x_fet_enable(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_FET_ENABLE);
}

/* Place the device in SEALED mode */
bq_status_t bq7690x_seal(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_SEAL);
}

/* Enter CONFIG_UPDATE mode */
bq_status_t bq7690x_set_config_update(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_SET_CFGUPDATE);
}

/* Exit CONFIG_UPDATE mode */
bq_status_t bq7690x_exit_config_update(bq7690x_t *dev)
{
    return bq_command_subcmd(dev, CMD_EXIT_CFGUPDATE);
}
