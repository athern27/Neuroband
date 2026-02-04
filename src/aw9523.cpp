#include "aw9523.h"

AW9523::AW9523(uint8_t addr) : _addr(addr), _wire(nullptr) {}

bool AW9523::begin(TwoWire *wire) {
    _wire = wire;
    
    // Small delay for chip power-up
    delay(10);
    
    // Perform soft reset first
    if (!reset()) {
        Serial.println("AW9523: Reset failed");
        // Continue anyway, some chips may not ACK reset
    }
    delay(10);  // Wait for reset to complete
    
    // Verify device ID
    uint8_t id = getDeviceID();
    if (id != AW9523_DEVICE_ID) {
        Serial.printf("AW9523: Invalid device ID 0x%02X (expected 0x%02X)\n", id, AW9523_DEVICE_ID);
        return false;
    }
    
    Serial.printf("AW9523: Found device at 0x%02X\n", _addr);
    
    // IMPORTANT: Set output registers FIRST to enable internal pull-ups
    // When a pin is configured as input, the output register controls the pull-up
    // 1 = pull-up enabled, 0 = pull-up disabled (floating)
    writeRegister(AW9523_REG_OUTPUT_P0, 0xFF);  // Enable pull-ups for all P0 pins
    writeRegister(AW9523_REG_OUTPUT_P1, 0xFF);  // Enable pull-ups for all P1 pins
    
    delay(10);  // Let pull-ups stabilize
    
    // Set GCR register - bit 4 controls Port 0 push-pull (1) vs open-drain (0)
    writeRegister(AW9523_REG_GCR, 0x10);  // Push-pull for P0
    
    // Set all pins to GPIO mode by default (not LED mode)
    // LED Mode register: 1 = GPIO mode, 0 = LED mode
    writeRegister(AW9523_REG_LED_MODE_P0, 0xFF);  // All P0 as GPIO
    writeRegister(AW9523_REG_LED_MODE_P1, 0xFF);  // All P1 as GPIO
    
    // Set all pins as inputs by default (Config: 1 = input, 0 = output)
    writeRegister(AW9523_REG_CONFIG_P0, 0xFF);  // All P0 as inputs
    writeRegister(AW9523_REG_CONFIG_P1, 0xFF);  // All P1 as inputs
    
    // Disable all interrupts by default (Int enable: 1 = disabled)
    writeRegister(AW9523_REG_INT_P0, 0xFF);
    writeRegister(AW9523_REG_INT_P1, 0xFF);
    
    delay(50);  // Wait for everything to stabilize
    
    return true;
}

bool AW9523::reset() {
    return writeRegister(AW9523_REG_RESET, 0x00);
}

uint8_t AW9523::getDeviceID() {
    return readRegister(AW9523_REG_ID);
}

void AW9523::pinToPortBit(uint8_t pin, uint8_t &port, uint8_t &bit) {
    if (pin < 8) {
        port = 0;
        bit = pin;
    } else {
        port = 1;
        bit = pin - 8;
    }
}

void AW9523::pinMode(uint8_t pin, uint8_t mode) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_CONFIG_P0 : AW9523_REG_CONFIG_P1;
    uint8_t config = readRegister(reg);
    
    if (mode == OUTPUT) {
        config &= ~(1 << bit);  // Clear bit = output
    } else {
        config |= (1 << bit);   // Set bit = input
    }
    
    writeRegister(reg, config);
}

void AW9523::setGPIOMode(uint8_t pin) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_LED_MODE_P0 : AW9523_REG_LED_MODE_P1;
    uint8_t mode = readRegister(reg);
    mode |= (1 << bit);  // Set bit = GPIO mode
    writeRegister(reg, mode);
}

void AW9523::setLEDMode(uint8_t pin) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_LED_MODE_P0 : AW9523_REG_LED_MODE_P1;
    uint8_t mode = readRegister(reg);
    mode &= ~(1 << bit);  // Clear bit = LED mode
    writeRegister(reg, mode);
}

void AW9523::digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1;
    uint8_t output = readRegister(reg);
    
    if (value == HIGH) {
        output |= (1 << bit);
    } else {
        output &= ~(1 << bit);
    }
    
    writeRegister(reg, output);
}

uint8_t AW9523::digitalRead(uint8_t pin) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_INPUT_P0 : AW9523_REG_INPUT_P1;
    uint8_t input = readRegister(reg);
    
    return (input & (1 << bit)) ? HIGH : LOW;
}

void AW9523::writePort(uint8_t port, uint8_t value) {
    uint8_t reg = (port == 0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1;
    writeRegister(reg, value);
}

uint8_t AW9523::readPort(uint8_t port) {
    uint8_t reg = (port == 0) ? AW9523_REG_INPUT_P0 : AW9523_REG_INPUT_P1;
    return readRegister(reg);
}

// Read all Port 0 inputs in one transaction (more reliable)
uint8_t AW9523::readPort0Inputs() {
    return readRegister(AW9523_REG_INPUT_P0);
}

// Read all Port 1 inputs in one transaction
uint8_t AW9523::readPort1Inputs() {
    return readRegister(AW9523_REG_INPUT_P1);
}

void AW9523::setLEDBrightness(uint8_t pin, uint8_t brightness) {
    // LED dimming register mapping:
    // P1_0-P1_3: DIM0-DIM3 (0x20-0x23)
    // P0_0-P0_7: DIM4-DIM11 (0x24-0x2B)
    // P1_4-P1_7: DIM12-DIM15 (0x2C-0x2F)
    
    uint8_t reg;
    if (pin < 8) {
        // Port 0 pins (P0_0 to P0_7)
        reg = AW9523_REG_DIM4 + pin;
    } else if (pin < 12) {
        // Port 1 pins 0-3 (P1_0 to P1_3)
        reg = AW9523_REG_DIM0 + (pin - 8);
    } else {
        // Port 1 pins 4-7 (P1_4 to P1_7)
        reg = AW9523_REG_DIM12 + (pin - 12);
    }
    
    writeRegister(reg, brightness);
}

void AW9523::enableInterrupt(uint8_t pin) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_INT_P0 : AW9523_REG_INT_P1;
    uint8_t intEn = readRegister(reg);
    intEn &= ~(1 << bit);  // Clear bit = interrupt enabled
    writeRegister(reg, intEn);
}

void AW9523::disableInterrupt(uint8_t pin) {
    uint8_t port, bit;
    pinToPortBit(pin, port, bit);
    
    uint8_t reg = (port == 0) ? AW9523_REG_INT_P0 : AW9523_REG_INT_P1;
    uint8_t intEn = readRegister(reg);
    intEn |= (1 << bit);  // Set bit = interrupt disabled
    writeRegister(reg, intEn);
}

bool AW9523::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0);
}

uint8_t AW9523::readRegister(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    if (_wire->available()) {
        return _wire->read();
    }
    return 0xFF;
}
