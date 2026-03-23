/*
 * drv8353.h
 *
 *  Created on: Dec 09, 2024
*/
#ifndef INC_DRV8353_H_
#define INC_DRV8353_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define DRV8353_REG_FAULT_STATUS_1      0x00
#define DRV8353_REG_VGS_STATUS_2        0x01
#define DRV8353_REG_DRIVER_CONTROL      0x02
#define DRV8353_REG_GATE_DRIVE_HS       0x03
#define DRV8353_REG_GATE_DRIVE_LS       0x04
#define DRV8353_REG_OCP_CONTROL         0x05
#define DRV8353_REG_CSA_CONTROL         0x06
#define DRV8353_REG_CAL_CONTROL         0x07

/* SPI Frame Format */
#define DRV8353_SPI_READ_CMD            0x8000  // Read command (bit 15 = 1)
#define DRV8353_SPI_WRITE_CMD           0x0000  // Write command (bit 15 = 0)
#define DRV8353_SPI_ADDR_MASK           0x7800  // Address bits [14:11]
#define DRV8353_SPI_DATA_MASK           0x07FF  // Data bits [10:0]

/* Driver Control Register (0x02) Bit Definitions */
#define DRV8353_CLR_FLT                 (1 << 0)   // Clear fault bits
#define DRV8353_BRAKE                   (1 << 1)   // Brake mode
#define DRV8353_COAST                   (1 << 2)   // Coast mode
#define DRV8353_PWM1_DIR                (1 << 4)   // PWM mode bit 0
#define DRV8353_PWM1_COM                (1 << 5)   // PWM mode bit 1
#define DRV8353_PWM_MODE_MASK           0x30       // PWM mode mask -
#define DRV8353_OTW_REP                 (1 << 6)   // Overtemp warning report
#define DRV8353_DIS_GDF                 (1 << 7)   // Disable gate drive fault
#define DRV8353_DIS_CPUV                (1 << 8)   // Disable charge pump UVLO
#define DRV8353_OCP_ACT                 (1 << 9)   // OCP action
#define DRV8353_DIS_CPUV_FLT            (1 << 10)  // Disable CP UVLO fault
#define DRV8353_RSVD_10                 (1 << 10)  // Reserved

/* Gate Drive HS Register (0x03) */
#define DRV8353_IDRIVEN_HS_MASK         0x000F  // HS gate drive sink current
#define DRV8353_IDRIVEP_HS_MASK         0x00F0  // HS gate drive source current
#define DRV8353_LOCK_MASK               0x0700  // Lock bits

/* Gate Drive LS Register (0x04) */
#define DRV8353_IDRIVEN_LS_MASK         0x000F  // LS gate drive sink current
#define DRV8353_IDRIVEP_LS_MASK         0x00F0  // LS gate drive source current
#define DRV8353_TDRIVE_MASK             0x0300  // Peak gate-current time
#define DRV8353_CBC                     (1 << 10) // Cycle-by-cycle mode

/* OCP Control Register (0x05) */
#define DRV8353_VDS_LVL_MASK            0x000F  // VDS trip level
#define DRV8353_OCP_DEG_MASK            0x0030  // OCP deglitch time
#define DRV8353_OCP_MODE_MASK           0x00C0  // OCP mode
#define DRV8353_DEAD_TIME_MASK          0x0300  // Dead time
#define DRV8353_TRETRY                  (1 << 10) // Retry time

/* CSA Control Register (0x06) */
#define DRV8353_SEN_LVL_MASK            0x0003  // Sense OCP level
#define DRV8353_CSA_CAL_A               (1 << 2)  // Calibrate CSA A
#define DRV8353_CSA_CAL_B               (1 << 3)  // Calibrate CSA B
#define DRV8353_CSA_CAL_C               (1 << 4)  // Calibrate CSA C
#define DRV8353_DIS_SEN                 (1 << 5)  // Disable sense OCP
#define DRV8353_CSA_GAIN_MASK           0x00C0  // CSA gain (00=5V/V, 01=10V/V, 10=20V/V, 11=40V/V)
#define DRV8353_LS_REF                  (1 << 8)  // CSA reference voltage
#define DRV8353_VREF_DIV                (1 << 9)  // VREF divider
#define DRV8353_CSA_FET                 (1 << 10) // CSA positive input

/* Fault Status 1 Register (0x00) */
#define DRV8353_FAULT                   (1 << 10) // Fault indicator
#define DRV8353_VDS_OCP                 (1 << 9)  // VDS overcurrent
#define DRV8353_GDF                     (1 << 8)  // Gate drive fault
#define DRV8353_UVLO                    (1 << 7)  // Undervoltage lockout
#define DRV8353_OTSD                    (1 << 6)  // Overtemperature shutdown
#define DRV8353_VDS_HA                  (1 << 5)  // VDS fault high-side A
#define DRV8353_VDS_LA                  (1 << 4)  // VDS fault low-side A
#define DRV8353_VDS_HB                  (1 << 3)  // VDS fault high-side B
#define DRV8353_VDS_LB                  (1 << 2)  // VDS fault low-side B
#define DRV8353_VDS_HC                  (1 << 1)  // VDS fault high-side C
#define DRV8353_VDS_LC                  (1 << 0)  // VDS fault low-side C

/* VGS Status 2 Register (0x01) */
#define DRV8353_SA_OC                   (1 << 10) // Sense amplifier A overcurrent
#define DRV8353_SB_OC                   (1 << 9)  // Sense amplifier B overcurrent
#define DRV8353_SC_OC                   (1 << 8)  // Sense amplifier C overcurrent
#define DRV8353_OTW                     (1 << 7)  // Overtemperature warning
#define DRV8353_CPUV                    (1 << 6)  // Charge pump undervoltage
#define DRV8353_VGS_HA                  (1 << 5)  // VGS fault high-side A
#define DRV8353_VGS_LA                  (1 << 4)  // VGS fault low-side A
#define DRV8353_VGS_HB                  (1 << 3)  // VGS fault high-side B
#define DRV8353_VGS_LB                  (1 << 2)  // VGS fault low-side B
#define DRV8353_VGS_HC                  (1 << 1)  // VGS fault high-side C
#define DRV8353_VGS_LC                  (1 << 0)  // VGS fault low-side C




/* DRV8353 Driver Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint16_t fault_status_1;
    uint16_t vgs_status_2;
    bool initialized;
} DRV8353_Handle;

HAL_StatusTypeDef DRV8353_ReadRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t *data);
HAL_StatusTypeDef DRV8353_WriteRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t data);
HAL_StatusTypeDef DRV8353_ReadFaults(DRV8353_Handle *drv);
void DRV8353_PrintFaults(DRV8353_Handle *drv);
HAL_StatusTypeDef DRV8353_Enable(DRV8353_Handle *drv);
HAL_StatusTypeDef DRV8353_Disable(DRV8353_Handle *drv);
void DRV8353_CalibrateCSAs(DRV8353_Handle *drv);
void DRV8353_RawSPITest(DRV8353_Handle *drv);

#endif /* INC_DRV8353_H_ */
