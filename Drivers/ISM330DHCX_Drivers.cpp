#define I2C_DEV_ADDR_ISM330DHCX 						(uint8_t) 0x6AU //0b01101010U

#define ISM330DHCX_ACC_REG_X_LOW						(uint8_t) 0x28U // register to read from for the acceleration
#define ISM330DHCX_GYRO_REG_X_LOW						(uint8_t) 0x22U // register to read from for the gyro
#define ISM330DHCX_TEMP_REG_LOW							(uint8_t) 0x20U // register to read from for the temperature
#define ISM330DHCX_ACC_ENABLE							(uint8_t) 0x6CU // instruction: 0x6C corresponds to 416Hz refresh rate and 8g range
#define ISM330DHCX_REG_CTRL1_ACC						(uint8_t) 0x10U // register to enable the acceleration
#define ISM330DHCX_GYRO_ENABLE							(uint8_t) 0x61U // instruction: 0x61 corresponds to 416Hz refresh rate and 4000dps range
#define ISM330DHCX_REG_CTRL2_GYRO						(uint8_t) 0x11U // register to enable the gyro

uint8_t beginI2CDevice(uint8_t device_addr) {
	Wire.beginTransmission(device_addr);
	return (uint8_t) Wire.endTransmission();
}

// read a register from the ISM330DHCX
void readFromISM330DHCX(uint8_t *buffer, uint8_t register_addr, uint8_t num_of_bytes_to_read) {
	Wire.beginTransmission(I2C_DEV_ADDR_ISM330DHCX);
	Wire.write(register_addr);
	Wire.endTransmission(false);
	
	Wire.requestFrom(I2C_DEV_ADDR_ISM330DHCX, num_of_bytes_to_read);
	
	uint8_t i = 0;
	while (Wire.available()) {
		buffer[i] = Wire.read();
		i++;
	}
}


// write to a register on the ISM330DHXC
void writeToISM330DHCX(uint8_t *buffer, uint8_t register_addr, uint8_t num_of_bytes_to_write) {
	Wire.beginTransmission(I2C_DEV_ADDR_ISM330DHCX);
	Wire.write(register_addr);
	
	for (uint8_t i = 0; i < num_of_bytes_to_write; i++) {
          Wire.write(buffer[i]);
        }

	Wire.endTransmission(false);
}