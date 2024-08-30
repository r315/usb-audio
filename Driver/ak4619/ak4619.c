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

static uint8_t ak4619_i2c_addr = 0;
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

static uint8_t ak4619_write_reg(uint16_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;

    return I2C_Master_Write(ak4619_i2c_addr, (const uint8_t*)&data, 2);
}

static uint8_t ak4619_read_reg(uint16_t reg, uint8_t *value)
{
    if(I2C_Master_Write(ak4619_i2c_addr, (const uint8_t*)&reg, 1) != 1)
        return 1;

    return I2C_Master_Read(ak4619_i2c_addr, value, 1);
}

static void ak4619_set_i2c_addr(uint8_t addr)
{
    if(addr != AK4619_I2C_ADDR && addr != AK4619_ALT_ADDR){
        if(ak4619_i2c_addr == 0)
            addr = AK4619_I2C_ADDR;
        return;
    }

    ak4619_i2c_addr = addr;
}

static void ak4619_dac_mux(uint8_t dac, uint8_t sel)
{
    uint8_t regVal;
    ak4619_read_reg(AK4619_DACDIN_REG, &regVal);

    if(dac > 1){
        sel = sel << 2;
        regVal &= ~(3 << 2);
    }else{
        regVal &= ~(3 << 0);
    }

    ak4619_write_reg(AK4619_DACDIN_REG, regVal | sel);
}


uint8_t ak4619_Init (uint8_t addr){

    uint8_t res;

    ak4619_set_i2c_addr(addr);

    res = ak4619_write_reg(AK4619_PWRMGM_REG, 0);  // Reset device

    if (!res){
        // Fail / not present
        return 0;
    }

    //delay_ms(100);
    res = ak4619_write_reg(AK4619_PWRMGM_REG, AK4619_PWRMGM_RST); // Set RST bit enables device

    if(!res){
         // Fail /
        return 0;
    }

    ak4619_Config(CDC_DEV_DAI, CDC_CFG_DAI_TDM);

    // Default MCLK = 256FS
    ak4619_write_reg(AK4619_SYSCLKSET_REG, AK4619_SYSCLKSET_256FS);

    ak4619_Enable();

    return 1;
}

uint8_t ak4619_Config (uint8_t DevID, uint8_t Cfg)
{
    switch(DevID)
    {
        case CDC_DEV_DAI:
            switch(Cfg){
                case CDC_CFG_DAI_SLOT_16BIT:
                case CDC_CFG_DAI_SLOT_24BIT:
                case CDC_CFG_DAI_SLOT_32BIT:
                case CDC_CFG_DAI_I2S:
                    ak4619_write_reg(AK4619_AUDFORM1_REG, AK4619_AUDFORM1_DSL16);
                    ak4619_write_reg(AK4619_AUDFORM2_REG, AK4619_AUDFORM2_DODL16 | AK4619_AUDFORM2_DIDL16);
                    break;

                case CDC_CFG_DAI_TDM:
                    ak4619_write_reg(AK4619_AUDFORM1_REG,
                                    AK4619_AUDFORM1_TDM |
                                    AK4619_AUDFORM1_DCF_TDM_MSB |
                                    AK4619_AUDFORM1_DSL16);

                    ak4619_write_reg(AK4619_AUDFORM2_REG,
                                    AK4619_AUDFORM2_SLOT |
                                    AK4619_AUDFORM2_DODL16 |
                                    AK4619_AUDFORM2_DIDL16);
                    break;
                default: break;
            }
            break;

        case CDC_DEV_DAC1:
        case CDC_DEV_DAC2:
        case CDC_DEV_DAC3:
        case CDC_DEV_DAC4:
            switch(Cfg){
                case AK4619_SEL_SDIN1:  ak4619_dac_mux(0, Cfg); break;
                case AK4619_SEL_SDIN2:  ak4619_dac_mux(1, Cfg); break;
                case AK4619_SEL_SDOUT1: ak4619_dac_mux(2, Cfg); break;
                case AK4619_SEL_SDOUT2: ak4619_dac_mux(3, Cfg); break;
                default: break;
            }
        break;

        case CDC_GET_MCLK:
            return CDC_MCLK_256FS;

        case CDC_SET_I2C_ADDR:
            ak4619_set_i2c_addr(Cfg);
            break;

        case CDC_GET_I2C_ADDR:
            return ak4619_i2c_addr;
    }

    return 0;
}

void ak4619_SampleRate (uint32_t Rate)
{
    // Set by MCLK
}

void ak4619_Enable (void)
{
    #if 0
    uint8_t value;

    if(!ak4619_read_reg(AK4619_PWRMGM_REG, &value)){
        return;
    }

    value |= AK4619_PWRMGM_PMDA1;

    ak4619_write_reg(AK4619_PWRMGM_REG, value);
    #else
    ak4619_write_reg(AK4619_PWRMGM_REG,
            AK4619_PWRMGM_PMDA1 |
            AK4619_PWRMGM_PMDA2 |
            AK4619_PWRMGM_PMAD1 |
            AK4619_PWRMGM_PMAD2 |
            AK4619_PWRMGM_RST );
    #endif
}

void ak4619_Disable (void)
{
    #if 0
    uint8_t value;

    if(!ak4619_read_reg(AK4619_PWRMGM_REG, &value)){
        return;
    }

    value &= ~AK4619_PWRMGM_PMDA1;

    ak4619_write_reg(AK4619_PWRMGM_REG, value);
    #else
    ak4619_write_reg(AK4619_PWRMGM_REG, AK4619_PWRMGM_RST);
    #endif
}

static void ak4619_dac_volume(uint8_t reg, uint8_t Volume)
{
    if(Volume > 100)
    {
        return;
    }

    Volume = (Volume * 255) / 100;

    ak4619_write_reg(reg, 0xFF - Volume);
}

// TODO: Check adc gain register range
static void ak4619_adc_gain(uint8_t reg, uint8_t Volume)
{
    if(Volume > 100)
    {
        return;
    }

    Volume = (Volume * 255) / 100;

    ak4619_write_reg(reg, Volume);
}


void ak4619_Volume (uint8_t DevID, uint8_t Volume)
{
    switch(DevID)
    {
        case CDC_DEV_ADC1: ak4619_adc_gain(AK4619_ADC1LVOL_REG, Volume); break;
        case CDC_DEV_ADC2: ak4619_adc_gain(AK4619_ADC1RVOL_REG, Volume); break;
        case CDC_DEV_ADC3: ak4619_adc_gain(AK4619_ADC2LVOL_REG, Volume); break;
        case CDC_DEV_ADC4: ak4619_adc_gain(AK4619_ADC2RVOL_REG, Volume); break;

        case CDC_DEV_DAC1: ak4619_dac_volume(AK4619_DAC1LVOL_REG, Volume); break;
        case CDC_DEV_DAC2: ak4619_dac_volume(AK4619_DAC1RVOL_REG, Volume); break;
        case CDC_DEV_DAC3: ak4619_dac_volume(AK4619_DAC2LVOL_REG, Volume); break;
        case CDC_DEV_DAC4: ak4619_dac_volume(AK4619_DAC2RVOL_REG, Volume); break;
        default : break;
    }
}

void ak4619_Mute (uint8_t DevID, uint8_t Mute)
{
    uint8_t regVal;

    ak4619_read_reg(AK4619_DACMUTFLT_REG, &regVal);

    switch(DevID){
        case CDC_DEV_DAC1:
        case CDC_DEV_DAC2:
            if(Mute){
               regVal |= AK4619_DACMUTFLT_DA1MUTE;
            }else{
                regVal &= ~AK4619_DACMUTFLT_DA1MUTE;
            }
            break;

        case CDC_DEV_DAC3:
        case CDC_DEV_DAC4:
            if(Mute){
               regVal |= AK4619_DACMUTFLT_DA2MUTE;
            }else{
                regVal &= ~AK4619_DACMUTFLT_DA2MUTE;
            }
            break;

        default: return;
    }

    ak4619_write_reg(AK4619_DACMUTFLT_REG, regVal);
}
