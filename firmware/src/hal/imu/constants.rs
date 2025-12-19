pub const BMM350_CHIP_ID: u8 = 0x33;
pub const BMM350_I2C_ADDR: u8 = 0x14;

// Soft reset command for BMM350
pub const BMM350_SOFT_RESET: u8 = 0xB6;

pub const BMI270_CHIP_ID: u8 = 0x24;
pub const BMI270_I2C_ADDR: u8 = 0x68;

/// BMI270 Register addresses
#[allow(dead_code)]
pub mod bmi270_reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const AUX_DATA_0: u8 = 0x04;
    pub const ACC_DATA_0: u8 = 0x0C;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const AUX_CONF: u8 = 0x44;
    pub const AUX_DEV_ID: u8 = 0x4B;
    pub const AUX_IF_CONF: u8 = 0x4C;
    pub const AUX_RD_ADDR: u8 = 0x4D;
    pub const AUX_WR_ADDR: u8 = 0x4E;
    pub const AUX_WR_DATA: u8 = 0x4F;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

/// BMM350 Register addresses (from BST-BMM350-DS001 datasheet)
pub mod bmm350_reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const PMU_CMD_AGGR_SET: u8 = 0x04;
    pub const PMU_CMD_AXIS_EN: u8 = 0x05;
    pub const PMU_CMD: u8 = 0x06;
    pub const PMU_CMD_STATUS_0: u8 = 0x07;
    // Magnetic data registers (24-bit per axis, little-endian)
    pub const MAG_X_XLSB: u8 = 0x31;
    pub const OTP_CMD_REG: u8 = 0x50;
    pub const CMD: u8 = 0x7E;
}

/// BMM350 PMU command values
pub mod bmm350_pmu {
    pub const NORMAL: u8 = 0x01;
    pub const UPDATE_OAE: u8 = 0x02;
    pub const FLUX_GUIDE_RESET: u8 = 0x05;
    pub const BIT_RESET: u8 = 0x07;
}

/// BMM350 ODR settings (bits 0-3 of PMU_CMD_AGGR_SET)
pub mod bmm350_odr {
    pub const ODR_100HZ: u8 = 0x04;
}

/// BMM350 Averaging settings (bits 4-5 of PMU_CMD_AGGR_SET)
pub mod bmm350_avg {
    pub const AVG_4: u8 = 0x02 << 4;
}
