#include "drv8353.h"
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

#define SPI_TIMEOUT 100


HAL_StatusTypeDef DRV8353_ReadRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t *data) {
    HAL_StatusTypeDef status;
    uint16_t tx_buffer;
    uint8_t tx_buf[2], rx_buf[2];

    // Construct read command: bit 15 = 1 (read), bits 14-11 = address
    tx_buffer = DRV8353_SPI_READ_CMD | ((reg_addr & 0x0F) << 11);

    //   bytes (MSB first)
    tx_buf[0] = (tx_buffer >> 8) & 0xFF;
    tx_buf[1] = tx_buffer & 0xFF;

    // CS low
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);

    // CS setup
    for(volatile uint32_t i = 0; i < 10; i++);

    status = HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, SPI_TIMEOUT);

    for(volatile uint32_t i = 0; i < 10; i++);

    //CS high
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        // Response comes in Frame 1 (during command transmission)
        uint16_t response_data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

        *data = response_data & 0x07FF;  // 11-bit data mask

        // Debug
        char msg[128];
        snprintf(msg, sizeof(msg), "Read reg 0x%02X: cmd=0x%04X resp=0x%04X\r\n",
                reg_addr, tx_buffer, response_data);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
    }

    return status;
}

HAL_StatusTypeDef DRV8353_WriteRegister(DRV8353_Handle *drv, uint8_t reg_addr, uint16_t data) {
    HAL_StatusTypeDef status;
    uint16_t tx_buffer;
    uint8_t tx_buf[2], rx_buf[2];

    // Construct write command: bit 15 = 0 (write), bits 14-11 = address, bits 10-0 = data
    tx_buffer = DRV8353_SPI_WRITE_CMD | ((reg_addr & 0x0F) << 11) | (data & DRV8353_SPI_DATA_MASK);

    tx_buf[0] = (tx_buffer >> 8) & 0xFF;
    tx_buf[1] = tx_buffer & 0xFF;

    // Pull CS low
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);

    for(volatile uint32_t i = 0; i < 10; i++);

    // Transmit write command
    status = HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, SPI_TIMEOUT);

    // Small delay before CS release
    for(volatile uint32_t i = 0; i < 10; i++);

    // Pull CS high
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        uint16_t response_data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

        char msg[128];
        snprintf(msg, sizeof(msg), "Write reg 0x%02X: 0x%04X (cmd=0x%04X, resp=0x%04X)\r\n",
                reg_addr, data, tx_buffer, response_data);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
    }

    return status;
}



HAL_StatusTypeDef DRV8353_ReadFaults(DRV8353_Handle *drv) {
    HAL_StatusTypeDef status;

    // Read Fault Status 1
    status = DRV8353_ReadRegister(drv, DRV8353_REG_FAULT_STATUS_1, &drv->fault_status_1);
    if (status != HAL_OK) {
        return status;
    }

    // Read VGS Status 2
    status = DRV8353_ReadRegister(drv, DRV8353_REG_VGS_STATUS_2, &drv->vgs_status_2);

    return status;
}


void DRV8353_PrintFaults(DRV8353_Handle *drv) {
    char msg[256];
    int len;

    len = snprintf(msg, sizeof(msg), "DRV8353 Faults:\r\n");
    CDC_Transmit_FS((uint8_t *)msg, len);

    if (drv->fault_status_1 & DRV8353_FAULT) {
        len = snprintf(msg, sizeof(msg), "  FAULT bit set\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->fault_status_1 & DRV8353_VDS_OCP) {
        len = snprintf(msg, sizeof(msg), "  VDS Overcurrent\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->fault_status_1 & DRV8353_GDF) {
        len = snprintf(msg, sizeof(msg), "  Gate Drive Fault\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->fault_status_1 & DRV8353_UVLO) {
        len = snprintf(msg, sizeof(msg), "  Undervoltage Lockout\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->fault_status_1 & DRV8353_OTSD) {
        len = snprintf(msg, sizeof(msg), "  Overtemperature Shutdown\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->vgs_status_2 & DRV8353_OTW) {
        len = snprintf(msg, sizeof(msg), "  Overtemperature Warning\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
    if (drv->vgs_status_2 & DRV8353_CPUV) {
        len = snprintf(msg, sizeof(msg), "  Charge Pump Undervoltage\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }

    if (!(drv->fault_status_1 & DRV8353_FAULT)) {
        len = snprintf(msg, sizeof(msg), "  No faults detected\r\n");
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}



HAL_StatusTypeDef DRV8353_Enable(DRV8353_Handle *drv) {
    HAL_StatusTypeDef status;
    uint16_t reg_data;

    status = DRV8353_ReadRegister(drv, DRV8353_REG_DRIVER_CONTROL, &reg_data);
    if (status != HAL_OK) {
        return status;
    }

    // Clear brake and coast bits
    reg_data &= ~(DRV8353_BRAKE | DRV8353_COAST);

    status = DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, reg_data);

    return status;
}


HAL_StatusTypeDef DRV8353_Disable(DRV8353_Handle *drv) {
    HAL_StatusTypeDef status;
    uint16_t reg_data;

    status = DRV8353_ReadRegister(drv, DRV8353_REG_DRIVER_CONTROL, &reg_data);
    if (status != HAL_OK) {
        return status;
    }

    // Set coast bit
    reg_data |= DRV8353_COAST;

    status = DRV8353_WriteRegister(drv, DRV8353_REG_DRIVER_CONTROL, reg_data);

    return status;
}



void DRV8353_CalibrateCSAs(DRV8353_Handle *drv)
{
    uint16_t csa_ctrl;
    char msg[128];

    // Read current CSA_CTRL
    DRV8353_ReadRegister(drv, DRV8353_REG_CAL_CONTROL, &csa_ctrl);

    //Set CAL_MODE = 1
    csa_ctrl |= 0x0001;  // Bit0 = 1
    DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, csa_ctrl);

    snprintf(msg, sizeof(msg), "DRV8353: Starting CSA Auto-Calibration...\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

    HAL_Delay(10);

    // Clear CAL_MODE = 0 to return to normal operation
    csa_ctrl &= ~0x0001;
    DRV8353_WriteRegister(drv, DRV8353_REG_CAL_CONTROL, csa_ctrl);

    snprintf(msg, sizeof(msg), "DRV8353: CSA Auto-Calibration Complete.\r\n");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void DRV8353_RawSPITest(DRV8353_Handle *drv) {
    char msg[256];
    uint8_t tx_buf[4], rx_buf[4];

    // Read Register 0x02
    tx_buf[0] = 0x90;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    for(volatile uint32_t i = 0; i < 20; i++);
    HAL_SPI_TransmitReceive(drv->hspi, tx_buf, rx_buf, 2, 100);
    for(volatile uint32_t i = 0; i < 20; i++);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    snprintf(msg, sizeof(msg), "  Response: [0x%02X 0x%02X]\r\n", rx_buf[0], rx_buf[1]);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}
