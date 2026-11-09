#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

bq_status_t bq7690x_deep_sleep(bq7690x_t *dev);        // Must send twice
bq_status_t bq7690x_exit_deep_sleep(bq7690x_t *dev);
bq_status_t bq7690x_shutdown(bq7690x_t *dev);          // Auto sends twice
bq_status_t bq7690x_sleep_enable(bq7690x_t *dev);
bq_status_t bq7690x_sleep_disable(bq7690x_t *dev);
