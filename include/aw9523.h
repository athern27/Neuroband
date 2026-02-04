#ifndef AW9523_H
#define AW9523_H

#include <Arduino.h>
#include <Wire.h>

// ==================== AW9523 I2C Addresses ====================
// Address determined by AD1 and AD0 pins:
// AD1=0, AD0=0 -> 0x58
// AD1=0, AD0=1 -> 0x59
// AD1=1, AD0=0 -> 0x5A
// AD1=1, AD0=1 -> 0x5B
#define AW9523_DEFAULT_ADDR 0x58

// ==================== AW9523 Register Addresses ====================
#define AW9523_REG_INPUT_P0     0x00  // Input port 0 (read-only)
#define AW9523_REG_INPUT_P1     0x01  // Input port 1 (read-only)
#define AW9523_REG_OUTPUT_P0    0x02  // Output port 0
#define AW9523_REG_OUTPUT_P1    0x03  // Output port 1
#define AW9523_REG_CONFIG_P0    0x04  // Config port 0 (0=output, 1=input)
#define AW9523_REG_CONFIG_P1    0x05  // Config port 1 (0=output, 1=input)
#define AW9523_REG_INT_P0       0x06  // Interrupt enable port 0
#define AW9523_REG_INT_P1       0x07  // Interrupt enable port 1
#define AW9523_REG_ID           0x10  // Device ID register (should read 0x23)
#define AW9523_REG_CTL          0x11  // Global control register
#define AW9523_REG_GCR          0x11  // GCR register (same as CTL) - bit4=P0 push-pull
#define AW9523_REG_LED_MODE_P0  0x12  // LED mode switch port 0 (0=LED, 1=GPIO)
#define AW9523_REG_LED_MODE_P1  0x13  // LED mode switch port 1 (0=LED, 1=GPIO)

// LED current control registers (P1_0 to P1_7, P0_0 to P0_7)
#define AW9523_REG_DIM0         0x20  // P1_0 LED current
#define AW9523_REG_DIM1         0x21  // P1_1 LED current
#define AW9523_REG_DIM2         0x22  // P1_2 LED current
#define AW9523_REG_DIM3         0x23  // P1_3 LED current
#define AW9523_REG_DIM4         0x24  // P0_0 LED current
#define AW9523_REG_DIM5         0x25  // P0_1 LED current
#define AW9523_REG_DIM6         0x26  // P0_2 LED current
#define AW9523_REG_DIM7         0x27  // P0_3 LED current
#define AW9523_REG_DIM8         0x28  // P0_4 LED current
#define AW9523_REG_DIM9         0x29  // P0_5 LED current
#define AW9523_REG_DIM10        0x2A  // P0_6 LED current
#define AW9523_REG_DIM11        0x2B  // P0_7 LED current
#define AW9523_REG_DIM12        0x2C  // P1_4 LED current
#define AW9523_REG_DIM13        0x2D  // P1_5 LED current
#define AW9523_REG_DIM14        0x2E  // P1_6 LED current
#define AW9523_REG_DIM15        0x2F  // P1_7 LED current

#define AW9523_REG_RESET        0x7F  // Software reset register

// Device ID
#define AW9523_DEVICE_ID        0x23

class AW9523 {
public:
    // Constructor
    AW9523(uint8_t addr = AW9523_DEFAULT_ADDR);
    
    // Initialize the AW9523 (call after Wire.begin())
    bool begin(TwoWire *wire = &Wire);
    
    // Software reset
    bool reset();
    
    // Get device ID (should return 0x23)
    uint8_t getDeviceID();
    
    // ==================== GPIO Configuration ====================
    
    // Set pin mode (0-15, where 0-7 = Port0, 8-15 = Port1)
    // mode: INPUT or OUTPUT
    void pinMode(uint8_t pin, uint8_t mode);
    
    // Set pin to GPIO mode (not LED mode)
    void setGPIOMode(uint8_t pin);
    
    // Set pin to LED mode (for PWM dimming)
    void setLEDMode(uint8_t pin);
    
    // ==================== GPIO Read/Write ====================
    
    // Write to a pin (0-15)
    void digitalWrite(uint8_t pin, uint8_t value);
    
    // Read from a pin (0-15)
    uint8_t digitalRead(uint8_t pin);
    
    // Write to entire port (0 or 1)
    void writePort(uint8_t port, uint8_t value);
    
    // Read entire port (0 or 1)
    uint8_t readPort(uint8_t port);
    
    // Read Port 0 inputs directly (for button reading)
    uint8_t readPort0Inputs();
    
    // Read Port 1 inputs directly (for button reading)
    uint8_t readPort1Inputs();
    
    // ==================== LED Dimming ====================
    
    // Set LED brightness (0-255) for a pin in LED mode
    void setLEDBrightness(uint8_t pin, uint8_t brightness);
    
    // ==================== Interrupt Configuration ====================
    
    // Enable interrupt for a pin
    void enableInterrupt(uint8_t pin);
    
    // Disable interrupt for a pin
    void disableInterrupt(uint8_t pin);
    
    // ==================== Low-level Register Access ====================
    
    // Write to a register
    bool writeRegister(uint8_t reg, uint8_t value);
    
    // Read from a register
    uint8_t readRegister(uint8_t reg);

private:
    uint8_t _addr;
    TwoWire *_wire;
    
    // Helper to get port and bit from pin number
    void pinToPortBit(uint8_t pin, uint8_t &port, uint8_t &bit);
};

#endif // AW9523_H
