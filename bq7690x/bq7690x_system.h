#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* Reset the device */
bq_status_t bq7690x_reset(bq7690x_t *dev);

/* Reset the accumulated charge and timer */
bq_status_t bq7690x_reset_pass_q(bq7690x_t *dev);

/* Toggle the FET_EN bit */
bq_status_t bq7690x_fet_enable(bq7690x_t *dev);

/* Place the device in SEALED mode */
bq_status_t bq7690x_seal(bq7690x_t *dev);

/* Enter CONFIG_UPDATE mode */
bq_status_t bq7690x_set_config_update(bq7690x_t *dev);

/* Exit CONFIG_UPDATE mode */
bq_status_t bq7690x_exit_config_update(bq7690x_t *dev);
