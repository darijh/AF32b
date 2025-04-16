
#include <AinHandler.h>

// Vout_A
HW_VOUT_A.SetReadChannel(ADC_CHANNEL_0);
HW_VOUT_A.SetEnable(true);
HW_VOUT_A.SetHiLimitEnable(true);
HW_VOUT_A.SetLoLimitEnable(true);
HW_VOUT_A.SetHiLimit(regs[MB_VHL_A]);
HW_VOUT_A.SetLoLimit(regs[MB_VLL_A]);
HW_VOUT_A.SetConstants(VOUT_MULT_A, VOUT_OFFSET_A);

// Vout_B
HW_VOUT_B.SetReadChannel(ADC_CHANNEL_4);
HW_VOUT_B.SetEnable(true);
HW_VOUT_B.SetHiLimitEnable(true);
HW_VOUT_B.SetLoLimitEnable(true);
HW_VOUT_B.SetHiLimit(regs[MB_VHL_B]);
HW_VOUT_B.SetLoLimit(regs[MB_VLL_B]);
HW_VOUT_B.SetConstants(VOUT_MULT_B, VOUT_OFFSET_B);

// Iout_A
HW_IOUT_A.SetReadChannel(ADC_CHANNEL_1);
HW_IOUT_A.SetEnable(true);
HW_IOUT_A.SetHiLimitEnable(true);
HW_IOUT_A.SetLoLimitEnable(true);
HW_IOUT_A.SetHiLimit(regs[MB_IHL_A]);
HW_IOUT_A.SetLoLimit(regs[MB_ILL_A]);
HW_IOUT_A.SetConstants(IOUT_MULT_A, IOUT_OFFSET_A);

// Iout_B
HW_IOUT_B.SetReadChannel(ADC_CHANNEL_8);
HW_IOUT_B.SetEnable(true);
HW_IOUT_B.SetHiLimitEnable(true);
HW_IOUT_B.SetLoLimitEnable(true);
HW_IOUT_B.SetHiLimit(regs[MB_IHL_B]);
HW_IOUT_B.SetLoLimit(regs[MB_ILL_B]);
HW_IOUT_B.SetConstants(IOUT_MULT_A, IOUT_OFFSET_A);
