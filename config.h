#ifndef CONFIG__H
#define CONFIG__H

// PIC32MK1024GPE100 Configuration Bit Settings

// 'C' source line config statements

// System frequency:
// SPLL => 4MHz
// f = 4MHz / FPLLIDIV * FPLLMULT / FPLLODIV
//   = 4MHz / 1 * 30 / 2
//   = 4MHz * 25
//   = 100MHz
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_40        // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)

// Turn off timers and features not used.
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Enabled)
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is enabled)
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Port Enabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ

#endif  // CONFIG__H
