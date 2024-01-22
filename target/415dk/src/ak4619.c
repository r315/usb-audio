#include "ak4619.h"
#include "board.h"

#define ADC1L_EN     (1 << 0)
#define ADC1R_EN     (1 << 1)
#define ADC2L_EN     (1 << 2)
#define ADC2R_EN     (1 << 3)

#define DAC1L_EN     (1 << 4)
#define DAC1R_EN     (1 << 5)
#define DAC2L_EN     (1 << 6)
#define DAC2R_EN     (1 << 7)

static uint8_t ak4619_write_reg(uint16_t, uint8_t);
static uint8_t ak4619_read_reg(uint16_t, uint8_t*);

const audio_codec_t ak4619 = {
    ak4619_Init,
    ak4619_Config,
    ak4619_SampleRate,
    ak4619_Enable,
    ak4619_Disable,
    ak4619_Volume,
    ak4619_Mute,
    ak4619_write_reg,
    ak4619_read_reg
};

static uint32_t ak4619_flags;

static uint8_t ak4619_write_reg(uint16_t reg, uint8_t value)
{
    reg = (value << 8 ) | reg;
    return I2C_Master_Write(AK4619_ADDR, (const uint8_t*)&reg, 2);
}

static uint8_t ak4619_read_reg(uint16_t reg, uint8_t *value)
{
    if(I2C_Master_Write(AK4619_ADDR, (const uint8_t*)&reg, 1) != 1)
        return 1;

    return I2C_Master_Read(AK4619_ADDR, value, 1);
}


uint8_t ak4619_Init (void){

    uint8_t res = ak4619_write_reg(AK4619_PWRMGM_REG, 0);  // Reset device

    if (!res){
        // Fail / not present
        return 0;
    }
    
    delay_ms(100);
    res = ak4619_write_reg(AK4619_PWRMGM_REG, AK4619_PWRMGM_RST); // Set RST bit enables device

    if(!res){
         // Fail / 
        return 0;
    }

    ak4619_flags = 0;

    return 1; 
}

uint8_t ak4619_Config (uint8_t DevID, uint8_t Cfg)
{
    return 1;
}

void ak4619_SampleRate (uint32_t Rate)
{
    
}

void ak4619_Enable (void) 
{
    uint8_t value;
    
    if(!ak4619_read_reg(AK4619_PWRMGM_REG, &value)){
        return;
    }

    value |= AK4619_PWRMGM_PMDA1;

    ak4619_write_reg(AK4619_PWRMGM_REG, value);
}

void ak4619_Disable (void) 
{
     uint8_t value;
    
    if(!ak4619_read_reg(AK4619_PWRMGM_REG, &value)){
        return;
    }

    value &= ~AK4619_PWRMGM_PMDA1;

    ak4619_write_reg(AK4619_PWRMGM_REG, value);
}

void ak4619_Volume (uint8_t DevID, uint8_t Volume){}
void ak4619_Mute (uint8_t DevID, uint8_t Mode) {}
