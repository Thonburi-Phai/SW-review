#pragma once
/*
 * BQ7690x register map. Device: BQ76907 / BQ76905.
 * @author: MV
 */

#define CRC_Mode 0    // 0 for disabled, 1 for enabled
#define DEV_ADDR 0x08 // BQ7690x i2c address is 0x08
#define R 0           // Read
#define W 1           // Write
#define W2 2          // write data with two bytes

// I2C direct addresses for transfer buffer and checksum
#define CMD_ADDR_LO            0x3E
#define CMD_ADDR_HI            0x3F
#define XFER_BUF_BASE          0x40    // 0x40..0x5F (32 bytes)
#define XFER_CHECKSUM          0x60
#define XFER_LENGTH            0x61

// Direct Commands - TRM 11.1.1
#define SafetyAlertA 0x02           // R H1 Alert flags for enabled safety alerts
#define SafetyStatusA 0x03          // R H1 Fault flags for enabled safety alerts
#define SafetyAlertB 0x04           // R H1 Alert flags for enabled safety alerts
#define SafetyStatusB 0x05          // R H1 Fault flags for enabled safety alerts
#define BatteryStatus 0x12          // R H2 Battery status flags
#define Cell1Voltage 0x14           // R I2 Cell1 voltage
#define Cell2Voltage 0x16           // R I2 Cell2 voltage
#define Cell3Voltage 0x18           // R I2 Cell3 voltage
#define Cell4Voltage 0x1A           // R I2 Cell4 voltage
#define Cell5Voltage 0x1C           // R I2 Cell5 voltage
#define Cell6Voltage 0x1E           // R I2 Cell6 voltage
#define Cell7Voltage 0x20           // R I2 Cell7 voltage
#define REG18Voltage 0x22           // R I2 Internal 1.8V regulator voltage via bandgap ref; VREF1/VREF2 diagnostic
#define VSSVoltage 0x24             // R I2 VSS via ADC; ADC input mux diagnostic
#define StackVoltage 0x26           // R U2 Top-of-stack voltage
#define IntTemperature 0x28         // R I2 Internal die temperature
#define TS1Temperature 0x2A         // R I2 TS pin ADC measurement
#define RawCurrent 0x36             // R I4 Raw current
#define Current 0x3A                // R I2 CC2 current
#define CC1Current 0x3C             // R I2 CC1 current
#define AlarmStatus 0x62            // RW H2 Latched flags asserting the ALERT pin
#define AlarmRawStatus 0x64         // R H2 Unlatched form of the AlarmStatus flags
#define AlarmEnable 0x66            // RW H2 Mask for AlarmStatus; writable during operation
#define FETControl 0x68             // RW H1 Host control of individual FET drivers (FETCONTROL in TRM)
#define REGOUTControl 0x69          // RW H1 Voltage regulator settings (REGOUTCONTROL in TRM)
#define DSGFETDriverPWMControl 0x6A // RW H2 DSG FET driver PWM mode
#define CHGFETDriverPWMControl 0x6C // RW H2 CHG FET driver PWM mode

// Command-only Subcommands - TRM 11.3
#define CMD_RESET_PASSQ 0x0005    // W Reset accumulated charge and timer
#define CMD_EXIT_DEEPSLEEP 0x000E // W Exit DEEPSLEEP
#define CMD_DEEPSLEEP 0x000F      // W Enter DEEPSLEEP; send twice within 4s
#define CMD_SHUTDOWN 0x0010       // W Start SHUTDOWN sequence; send twice within 4s
#define CMD_RESET 0x0012          // W Reset device; not available in SEALED
#define CMD_FET_ENABLE 0x0022     // W Toggle FET_EN; not available in SEALED
#define CMD_SEAL 0x0030           // W Enter SEALED mode; not available in SEALED
#define CMD_SET_CFGUPDATE 0x0090  // W Enter CONFIG_UPDATE mode; not available in SEALED
#define CMD_EXIT_CFGUPDATE 0x0092 // W Exit CONFIG_UPDATE mode; not available in SEALED
#define CMD_SLEEP_ENABLE 0x0099   // W Allow SLEEP mode
#define CMD_SLEEP_DISABLE 0x009A  // W Block SLEEP mode

// Subcommands - TRM 11.4
#define DEVICE_NUMBER 0x0001   // R H2 Device number, little-endian
#define FW_VERSION 0x0002      // R H6 Three 16-bit words; see TRM
#define HW_VERSION 0x0003      // R H2 Hardware version
#define PASSQ 0x0004           // R Offset 0 U4, offset 4 I4, offset 8 U4; see TRM
#define SECURITY_KEYS 0x0035   // W H4 Key for SEALED to FULLACCESS; not available in SEALED
#define CB_ACTIVE_CELLS 0x0083 // RW H1 (R in SEALED) Bit mask of actively balanced cells
#define PROG_TIMER 0x0094      // RW H2 (R in SEALED) Timer disabling the REGOUT LDO until timeout or alarm
#define PROT_RECOVERY 0x009B   // RW H1 (R in SEALED) Allow recovery of selected protection faults

// Data Memory register addresses (BQ7690x TRM)
#define CellGain 0x9000                                       // U2 Calibration:Voltage:Cell Gain
#define StackGain 0x9002                                      // U2 Calibration:Voltage:Stack Gain
#define Cell2Offset 0x9004                                    // I1 Calibration:Voltage:Cell Offset
#define Cell3Offset 0x9005                                    // I1 Calibration:Voltage:Cell Offset
#define Cell45Offset 0x9071                                   // I1 Calibration:Voltage:Cell Offset
#define Cell67Offset 0x9072                                   // I1 Calibration:Voltage:Cell Offset

#define CurrGain 0x9006                                       // U2 Calibration:Current:Curr Gain
#define CurrOffset 0x9008                                     // I2 Calibration:Current:Curr Offset
#define CC1Gain 0x900a                                        // U2 Calibration:Current:CC1 Gain
#define CC1Offset 0x900c                                      // I2 Calibration:Current:CC1 Offset
#define TSOffset 0x900e                                       // I2 Calibration:Temperature:TS Offset
#define IntTempGain 0x9010                                    // U2 Calibration:Temperature:Int Temp Gain
#define IntTempOffset 0x9012                                  // I2 Calibration:Temperature:Int Temp Offset
#define PowerConfig 0x9014                                    // H1 Settings:Configuration:Power Config
#define REGOUTConfig 0x9015                                   // H1 Settings:Configuration:REGOUT Config
#define I2CAddress 0x9016                                     // H1 Settings:Configuration:I2C Address
#define I2CConfig 0x9017                                      // H2 Settings:Configuration:I2C Config
#define DAConfig 0x9019                                       // H2 Settings:Configuration:DA Config
#define VcellMode 0x901b                                      // H1 Settings:Configuration:Vcell Mode
#define DefaultAlarmMask 0x901c                               // H2 Settings:Configuration:Default Alarm Mask
#define FETOptions 0x901e                                     // H1 Settings:Configuration:FET Options
#define ChargeDetectorTime 0x901f                             // U1 Settings:Configuration:Charge Detector Time
#define BalancingConfiguration 0x9020                         // H1 Settings:Cell Balancing:Balancing Configuration
#define MinTempThreshold 0x9021                               // U1 Settings:Cell Balancing:Min Temp Threshold
#define MaxTempThreshold 0x9022                               // U1 Settings:Cell Balancing:Max Temp Threshold
#define MaxInternalTemp 0x9023                                // I1 Settings:Cell Balancing:Max Internal Temp
#define EnabledProtectionsA 0x9024                            // H1 Settings:Protection:Enabled Protections A
#define EnabledProtectionsB 0x9025                            // H1 Settings:Protection:Enabled Protections B
#define DSGFETProtectionsA 0x9026                             // H1 Settings:Protection:DSG FET Protections A
#define CHGFETProtectionsA 0x9027                             // H1 Settings:Protection:CHG FET Protections A
#define BothFETProtectionsB 0x9028                            // H1 Settings:Protection:Both FET Protections B
#define BodyDiodeThreshold 0x9029                             // I2 Settings:Protection:Body Diode Threshold
#define CellOpenWireNORMALCheckTime 0x902b                    // U1 Settings:Protection:Cell Open Wire NORMAL Check Time
#define CellOpenWireSLEEPCheckTime 0x902c                     // H1 Settings:Protection:Cell Open Wire SLEEP Check Time
#define HostWatchdogTimeout 0x902d                            // U1 Settings:Protection:Host Watchdog Timeout
#define CellUndervoltageProtectionThreshold 0x902e            // I2 Protections:Cell Voltage:Cell Undervoltage Protection Threshold
#define CellUndervoltageProtectionDelay 0x9030                // U1 Protections:Cell Voltage:Cell Undervoltage Protection Delay
#define CellUndervoltageProtectionRecoveryHysteresis 0x9031   // H1 Protections:Cell Voltage:Cell Undervoltage Protection Recovery Hysteresis
#define CellOvervoltageProtectionThreshold 0x9032             // I2 Protections:Cell Voltage:Cell Overvoltage Protection Threshold
#define CellOvervoltageProtectionDelay 0x9034                 // U1 Protections:Cell Voltage:Cell Overvoltage Protection Delay
#define CellOvervoltageProtectionRecoveryHysteresis 0x9035    // H1 Protections:Cell Voltage:Cell Overvoltage Protection Recovery Hysteresis
#define OvercurrentinChargeProtectionThreshold 0x9036         // U1 Protections:Current:Overcurrent in Charge Protection Threshold
#define OvercurrentinChargeProtectionDelay 0x9037             // U1 Protections:Current:Overcurrent in Charge Protection Delay
#define OvercurrentinDischarge1ProtectionThreshold 0x9038     // U1 Protections:Current:Overcurrent in Discharge 1 Protection Threshold
#define OvercurrentinDischarge1ProtectionDelay 0x9039         // U1 Protections:Current:Overcurrent in Discharge 1 Protection Delay
#define OvercurrentinDischarge2ProtectionThreshold 0x903a     // U1 Protections:Current:Overcurrent in Discharge 2 Protection Threshold
#define OvercurrentinDischarge2ProtectionDelay 0x903b         // U1 Protections:Current:Overcurrent in Discharge 2 Protection Delay
#define ShortCircuitinDischargeProtectionThreshold 0x903c     // H1 Protections:Current:Short Circuit in Discharge Protection Threshold
#define ShortCircuitinDischargeProtectionDelay 0x903d         // H1 Protections:Current:Short Circuit in Discharge Protection Delay
#define LatchLimit 0x903e                                     // H1 Protections:Current:Latch Limit
#define RecoveryTime 0x903f                                   // U1 Protections:Current:Recovery Time
#define OvertemperatureinChargeProtectionThreshold 0x9040     // U1 Protections:Temperature:Overtemperature in Charge Protection Threshold
#define OvertemperatureinChargeProtectionDelay 0x9041         // U1 Protections:Temperature:Overtemperature in Charge Protection Delay
#define OvertemperatureinChargeProtectionRecovery 0x9042      // U1 Protections:Temperature:Overtemperature in Charge Protection Recovery
#define UndertemperatureinChargeProtectionThreshold 0x9043    // U1 Protections:Temperature:Undertemperature in Charge Protection Threshold
#define UndertemperatureinChargeProtectionDelay 0x9044        // U1 Protections:Temperature:Undertemperature in Charge Protection Delay
#define UndertemperatureinChargeProtectionRecovery 0x9045     // U1 Protections:Temperature:Undertemperature in Charge Protection Recovery
#define OvertemperatureinDischargeProtectionThreshold 0x9046  // U1 Protections:Temperature:Overtemperature in Discharge Protection Threshold
#define OvertemperatureinDischargeProtectionDelay 0x9047      // U1 Protections:Temperature:Overtemperature in Discharge Protection Delay
#define OvertemperatureinDischargeProtectionRecovery 0x9048   // U1 Protections:Temperature:Overtemperature in Discharge Protection Recovery
#define UndertemperatureinDischargeProtectionThreshold 0x9049 // U1 Protections:Temperature:Undertemperature in Discharge Protection Threshold
#define UndertemperatureinDischargeProtectionDelay 0x904a     // U1 Protections:Temperature:Undertemperature in Discharge Protection Delay
#define UndertemperatureinDischargeProtectionRecovery 0x904b  // U1 Protections:Temperature:Undertemperature in Discharge Protection Recovery
#define InternalOvertemperatureProtectionThreshold 0x904c     // U1 Protections:Temperature:Internal Overtemperature Protection Threshold
#define InternalOvertemperatureProtectionDelay 0x904d         // U1 Protections:Temperature:Internal Overtemperature Protection Delay
#define InternalOvertemperatureProtectionRecovery 0x904e      // U1 Protections:Temperature:Internal Overtemperature Protection Recovery
#define SleepCurrent 0x904f                                   // I2 Power:Sleep:Sleep Current
#define VoltageTime 0x9051                                    // U1 Power:Sleep:Voltage Time
#define WakeComparatorCurrent 0x9052                          // U1 Power:Sleep:Wake Comparator Current
#define ShutdownCellVoltage 0x9053                            // I2 Power:Shutdown:Shutdown Cell Voltage
#define ShutdownStackVoltage 0x9055                           // U2 Power:Shutdown:Shutdown Stack Voltage
#define ShutdownTemperature 0x9057                            // U1 Power:Shutdown:Shutdown Temperature
#define AutoShutdownTime 0x9058                               // U1 Power:Shutdown:Auto Shutdown Time
#define SecuritySettings 0x9059                               // H1 Security:Settings:Security Settings
#define FullAccessKeyStep1 0x905a                             // H2 Security:Settings:Full Access Key Step 1
#define FullAccessKeyStep2 0x905c                             // H2 Security:Settings:Full Access Key Step 2
