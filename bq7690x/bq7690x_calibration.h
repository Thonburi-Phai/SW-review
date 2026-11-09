/**
 * @file bq7690x_calibration.h
 * @brief BQ7690x voltage, current and temperature calibration registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Voltage Calibration ========== */

/** Read cell voltage gain calibration (U2) */
bq_status_t bq7690x_read_cell_gain(bq7690x_t *dev, uint16_t *gain);

/** Write cell voltage gain calibration (U2) */
bq_status_t bq7690x_write_cell_gain(bq7690x_t *dev, uint16_t gain);

/** Read stack voltage gain calibration (U2) */
bq_status_t bq7690x_read_stack_gain(bq7690x_t *dev, uint16_t *gain);

/** Write stack voltage gain calibration (U2) */
bq_status_t bq7690x_write_stack_gain(bq7690x_t *dev, uint16_t gain);

/** Read cell 2 offset calibration (I1) */
bq_status_t bq7690x_read_cell2_offset(bq7690x_t *dev, int8_t *offset);

/** Write cell 2 offset calibration (I1) */
bq_status_t bq7690x_write_cell2_offset(bq7690x_t *dev, int8_t offset);

/** Read cell 3 offset calibration (I1) */
bq_status_t bq7690x_read_cell3_offset(bq7690x_t *dev, int8_t *offset);

/** Write cell 3 offset calibration (I1) */
bq_status_t bq7690x_write_cell3_offset(bq7690x_t *dev, int8_t offset);

/** Read cell 4-5 offset calibration (I1) */
bq_status_t bq7690x_read_cell45_offset(bq7690x_t *dev, int8_t *offset);

/** Write cell 4-5 offset calibration (I1) */
bq_status_t bq7690x_write_cell45_offset(bq7690x_t *dev, int8_t offset);

/** Read cell 6-7 offset calibration (I1) */
bq_status_t bq7690x_read_cell67_offset(bq7690x_t *dev, int8_t *offset);

/** Write cell 6-7 offset calibration (I1) */
bq_status_t bq7690x_write_cell67_offset(bq7690x_t *dev, int8_t offset);

/* ========== Current Calibration ========== */

/** Read CC2 current gain calibration (U2) */
bq_status_t bq7690x_read_curr_gain(bq7690x_t *dev, uint16_t *gain);

/** Write CC2 current gain calibration (U2) */
bq_status_t bq7690x_write_curr_gain(bq7690x_t *dev, uint16_t gain);

/** Read CC2 current offset calibration (I2) */
bq_status_t bq7690x_read_curr_offset(bq7690x_t *dev, int16_t *offset);

/** Write CC2 current offset calibration (I2) */
bq_status_t bq7690x_write_curr_offset(bq7690x_t *dev, int16_t offset);

/** Read CC1 current gain calibration (U2) */
bq_status_t bq7690x_read_cc1_gain(bq7690x_t *dev, uint16_t *gain);

/** Write CC1 current gain calibration (U2) */
bq_status_t bq7690x_write_cc1_gain(bq7690x_t *dev, uint16_t gain);

/** Read CC1 current offset calibration (I2) */
bq_status_t bq7690x_read_cc1_offset(bq7690x_t *dev, int16_t *offset);

/** Write CC1 current offset calibration (I2) */
bq_status_t bq7690x_write_cc1_offset(bq7690x_t *dev, int16_t offset);

/* ========== Temperature Calibration ========== */

/** Read external thermistor offset calibration (I2) */
bq_status_t bq7690x_read_ts_offset(bq7690x_t *dev, int16_t *offset);

/** Write external thermistor offset calibration (I2) */
bq_status_t bq7690x_write_ts_offset(bq7690x_t *dev, int16_t offset);

/** Read internal temperature gain calibration (U2) */
bq_status_t bq7690x_read_int_temp_gain(bq7690x_t *dev, uint16_t *gain);

/** Write internal temperature gain calibration (U2) */
bq_status_t bq7690x_write_int_temp_gain(bq7690x_t *dev, uint16_t gain);

/** Read internal temperature offset calibration (I2) */
bq_status_t bq7690x_read_int_temp_offset(bq7690x_t *dev, int16_t *offset);

/** Write internal temperature offset calibration (I2) */
bq_status_t bq7690x_write_int_temp_offset(bq7690x_t *dev, int16_t offset);
