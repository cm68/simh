/*************************************************************************
 *                                                                       *
 * $Id: s100_mmio.c 1997 2008-07-18 05:29:52Z hharte $                    *
 *                                                                       *
 * Copyright (c) 2007-2008 Howard M. Harte.                              *
 * http://www.hartetec.com                                               *
 *                                                                       *
 * Permission is hereby granted, free of charge, to any person obtaining *
 * a copy of this software and associated documentation files (the       *
 * "Software"), to deal in the Software without restriction, including   *
 * without limitation the rights to use, copy, modify, merge, publish,   *
 * distribute, sublicense, and/or sell copies of the Software, and to    *
 * permit persons to whom the Software is furnished to do so, subject to *
 * the following conditions:                                             *
 *                                                                       *
 * The above copyright notice and this permission notice shall be        *
 * included in all copies or substantial portions of the Software.       *
 *                                                                       *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                 *
 * NONINFRINGEMENT. IN NO EVENT SHALL HOWARD M. HARTE BE LIABLE FOR ANY  *
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  *
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     *
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                *
 *                                                                       *
 * Except as contained in this notice, the name of Howard M. Harte shall *
 * not be used in advertising or otherwise to promote the sale, use or   *
 * other dealings in this Software without prior written authorization   *
 * Howard M. Harte.                                                      *
 *                                                                       *
 * SIMH Interface based on altairz80_hdsk.c, by Peter Schorn.            *
 *                                                                       *
 * Module Description:                                                   *
 *     Morrow designs MIO and wunderbus I/O module for SIMH.
 * Note this does not include the Boot ROM on the System Support 1 Card  *
 *                                                                       *
 * Environment:                                                          *
 *     User mode only                                                    *
 *                                                                       *
 * derived from s100_ss1.c, as the system support 1 resembles the morrow *
 * mmio.                                                                 *
 *                                                                       *
 *************************************************************************/

/*#define DBG_MSG */

#include "altairz80_defs.h"

#ifdef DBG_MSG
#define DBG_PRINT(args) sim_printf args
#else
#define DBG_PRINT(args)
#endif

/* Debug flags */
#define ERROR_MSG   (1 << 0)
#define TRACE_MSG   (1 << 1)
#define PIC_MSG     (1 << 2)
#define TC_MSG      (1 << 3)
#define RTC_MSG     (1 << 4)
#define MATH_MSG    (1 << 5)
#define UART_MSG    (1 << 6)
#define IRQ_MSG     (1 << 7)

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
} MMIO_INFO;

static MMIO_INFO mmio_info_data = { { 0x0, 0, 0x50, 16 } };

extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);
extern uint32 PCX;

static t_stat mmio_reset(DEVICE *mmio_dev);
static t_stat mmio_svc (UNIT *uptr);
static uint8 MMIO_Read(const uint32 Addr);
static uint8 MMIO_Write(const uint32 Addr, uint8 cData);
static int32 mmiodev(const int32 port, const int32 io, const int32 data);
void raise_mmio_interrupt(uint8 isr_index);
static const char* mmio_description(DEVICE *dptr);
static void setClockmmio(void);

/* MMIO1 Interrupt Controller notes:
 *
 * Msster 8259:
 * IRQ0 = VI0
 * IRQ1 = VI1       - DISK3 Interrupt
 * IRQ2 = VI2       - IF3 Rx Interrupt
 * IRQ3 = VI3       - IF3 Tx Interrupt
 * IRQ4 = VI4       - DISK1A
 * IRQ5 = VI5       - ?
 * IRQ6 = VI6
 * <Cascade>
 *
 * Slave 8259:
 * IRQ0 = VI7           0x48
 * IRQ1 = Timer0        0x49
 * IRQ2 = Timer1        0x4A
 * IRQ3 = Timer2        0x4B
 * IRQ4 = 9511 SVRQ     0x4C
 * IRQ5 = 9511 END      0x4D
 * IRQ6 = 2651 TxRDY    0x4E
 * IRQ7 = 2651 RxRDY    0x4F
 */
#define MASTER_PIC  0
#define SLAVE_PIC   1

#define VI0_IRQ_OFFSET      0
#define VI1_IRQ_OFFSET      1
#define VI2_IRQ_OFFSET      2
#define VI3_IRQ_OFFSET      3
#define VI4_IRQ_OFFSET      4
#define VI5_IRQ_OFFSET      5
#define VI6_IRQ_OFFSET      6
#define VI7_IRQ_OFFSET      0
#define TC0_IRQ_OFFSET      1
#define TC1_IRQ_OFFSET      2
#define TC2_IRQ_OFFSET      3
#define MSVRQ_IRQ_OFFSET    4
#define MEND_IRQ_OFFSET     5
#define TXRDY_IRQ_OFFSET    6
#define RXRDY_IRQ_OFFSET    7

typedef struct {
    uint8 config_cnt;
    uint8 ICW[5];
    uint8 IMR;      /* OCW1 = IMR */
    uint8 OCW2;
    uint8 OCW3;
    uint8 IRR;
    uint8 ISR;
} I8259_REGS;

I8259_REGS mmio_pic[2];

/* SS1 Timer notes:
 *
 * T0, T1, T2 inputs connected to 2MHz clock on SS1
 * T0 IRQ connected to Slave IRQ 1
 * T1 IRQ connected to Slave IRQ 2
 * T2 IRQ connected to Slave IRQ 3
 */
typedef struct {
    uint16 count[3];    /* Current counter value for each timer. */
    uint8 mode[3];      /* Current mode of each timer. */
    uint8 bcd[3];
    uint8 rl[3];
    uint8 CTL;
} I8253_REGS;

I8253_REGS mmio_tc[1];

#define I8253_CTL_SC_MASK   0xC0
#define I8253_CTL_RL_MASK   0x30
#define I8253_CTL_MODE_MASK 0x0E
#define I8253_CTL_BCD       0x01

#define RTS_SECONDS_1_DIGIT     0
#define RTS_SECONDS_10_DIGIT    1
#define RTS_MINUTES_1_DIGIT     2
#define RTS_MINUTES_10_DIGIT    3
#define RTS_HOURS_1_DIGIT       4
#define RTS_HOURS_10_DIGIT      5
#define RTS_DAY_OF_WEEK_DIGIT   6
#define RTS_DAYS_1_DIGIT        7
#define RTS_DAYS_10_DIGIT       8
#define RTS_MONTHS_1_DIGIT      9
#define RTS_MONTHS_10_DIGIT     10
#define RTS_YEARS_1_DIGIT       11
#define RTS_YEARS_10_DIGIT      12

typedef struct {
    uint8 digit_sel;
    uint8 flags;
    uint8 digits[RTS_YEARS_10_DIGIT + 1];
    int32 clockDelta; /* delta between real clock and SS1 clock */
} RTC_REGS;

RTC_REGS mmio_rtc[1] = { { 0 } };

static UNIT mmio_unit[] = {
    { UDATA (&mmio_svc, UNIT_FIX | UNIT_DISABLE | UNIT_ROABLE, 0) },
    { UDATA (&mmio_svc, UNIT_FIX | UNIT_DISABLE | UNIT_ROABLE, 0) },
    { UDATA (&mmio_svc, UNIT_FIX | UNIT_DISABLE | UNIT_ROABLE, 0) },
    { UDATA (&mmio_svc, UNIT_FIX | UNIT_DISABLE | UNIT_ROABLE, 0) }
};

static REG mmio_reg[] = {
    { HRDATAD (MPIC_IMR,    mmio_pic[MASTER_PIC].IMR,     8,  "Master IMR register"),     },
    { HRDATAD (MPIC_IRR,    mmio_pic[MASTER_PIC].IRR,     8,  "Master IRR register"),     },
    { HRDATAD (MPIC_ISR,    mmio_pic[MASTER_PIC].ISR,     8,  "Master ISR register"),     },
    { HRDATAD (MPIC_OCW2,   mmio_pic[MASTER_PIC].OCW2,    8,  "Master OCW2 register"),    },
    { HRDATAD (MPIC_OCW3,   mmio_pic[MASTER_PIC].OCW3,    8,  "Master OCW3 register"),    },

    { HRDATAD (SPIC_IMR,    mmio_pic[SLAVE_PIC].IMR,      8,  "Slave IMR register"),      },
    { HRDATAD (SPIC_IRR,    mmio_pic[SLAVE_PIC].IRR,      8,  "Slave IRR register"),      },
    { HRDATAD (SPIC_ISR,    mmio_pic[SLAVE_PIC].ISR,      8,  "Slave ISR register"),      },
    { HRDATAD (SPIC_OCW2,   mmio_pic[SLAVE_PIC].OCW2,     8,  "Slave OCW2 register"),     },
    { HRDATAD (SPIC_OCW3,   mmio_pic[SLAVE_PIC].OCW3,     8,  "Slave OCW3 register"),     },

    { HRDATAD (T0_MODE,     mmio_tc[0].mode[0],           3,  "Timer 0 mode register"),   },
    { HRDATAD (T0_COUNT,    mmio_tc[0].count[0],          16, "Timer 0 count register"),  },
    { HRDATAD (T1_MODE,     mmio_tc[0].mode[1],           3,  "Timer 1 mode register"),   },
    { HRDATAD (T1_COUNT,    mmio_tc[0].count[1],          16, "Timer 1 count register"),  },
    { HRDATAD (T2_MODE,     mmio_tc[0].mode[2],           3,  "Timer 2 mode register"),   },
    { HRDATAD (T2_COUNT,    mmio_tc[0].count[2],          16, "Timer 2 count register"),  },

    { HRDATAD (RTC_DIGIT,   mmio_rtc[0].digit_sel,        4,  "Digit selector register"), },
    { HRDATAD (RTC_FLAGS,   mmio_rtc[0].flags,            4,  "Flags register"),          },
    { DRDATAD (RTC_DELTA,   mmio_rtc[0].clockDelta,      32,
               "SS1 Clock - Delta between real clock and SS1 clock")                        },
    { HRDATAD (RTC_DIGIT_SEC_1, mmio_rtc[0].digits[RTS_SECONDS_1_DIGIT],     4,  "Seconds 1 digit"),     },
    { HRDATAD (RTC_DIGIT_SEC_10,mmio_rtc[0].digits[RTS_SECONDS_10_DIGIT],    4,  "Seconds 10 digit"),    },
    { HRDATAD (RTC_DIGIT_MIN_1, mmio_rtc[0].digits[RTS_MINUTES_1_DIGIT],     4,  "Minutes 1 digit"),     },
    { HRDATAD (RTC_DIGIT_MIN_10,mmio_rtc[0].digits[RTS_MINUTES_10_DIGIT],    4,  "Minutes 10 digit"),    },
    { HRDATAD (RTC_DIGIT_HR_1,  mmio_rtc[0].digits[RTS_HOURS_1_DIGIT],       4,  "Hours 1 digit"),       },
    { HRDATAD (RTC_DIGIT_HR_10, mmio_rtc[0].digits[RTS_HOURS_10_DIGIT],      4,  "Hours 10 digit"),      },
    { HRDATAD (RTC_DIGIT_DAY,   mmio_rtc[0].digits[RTS_DAY_OF_WEEK_DIGIT],   4,  "Day of week digit"),   },
    { HRDATAD (RTC_DIGIT_DAY_1, mmio_rtc[0].digits[RTS_DAYS_1_DIGIT],        4,  "Days 1 digit"),        },
    { HRDATAD (RTC_DIGIT_DAY_10,mmio_rtc[0].digits[RTS_DAYS_10_DIGIT],       4,  "Days 10 digit"),       },
    { HRDATAD (RTC_DIGIT_MO_1,  mmio_rtc[0].digits[RTS_MONTHS_1_DIGIT],      4,  "Months 1 digit"),      },
    { HRDATAD (RTC_DIGIT_MO_10, mmio_rtc[0].digits[RTS_MONTHS_10_DIGIT],     4,  "Months 10 digit"),     },
    { HRDATAD (RTC_DIGIT_YR_1,  mmio_rtc[0].digits[RTS_YEARS_1_DIGIT],       4,  "Years 1 digit"),       },
    { HRDATAD (RTC_DIGIT_YR_10, mmio_rtc[0].digits[RTS_YEARS_10_DIGIT],      4,  "Years 10 digit"),      },

    { NULL }
};

static const char* mmio_description(DEVICE *dptr) {
    return "Compupro System Support 1";
}

static MTAB mmio_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0,              "IOBASE",   "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets system support module base address" },
    { 0 }
};

/* Debug Flags */
static DEBTAB mmio_dt[] = {
    { "ERROR",  ERROR_MSG,  "Error messages"    },
    { "TRACE",  TRACE_MSG,  "Trace messages"    },
    { "PIC",    PIC_MSG,    "PIC messages"      },
    { "TC",     TC_MSG,     "TC messages"       },
    { "RTC",    RTC_MSG,    "RTC messages"      },
    { "MATH",   MATH_MSG,   "Math messages"     },
    { "UART",   UART_MSG,   "UART messages"     },
    { "IRQ",    IRQ_MSG,    "IRQ messages"      },
    { NULL,     0                               }
};

DEVICE mmio_dev = {
    "SS1", mmio_unit, mmio_reg, mmio_mod,
    SS1_MAX_TIMERS, 10, 31, 1, SS1_MAX_TIMERS, SS1_MAX_TIMERS,
    NULL, NULL, &mmio_reset,
    NULL, NULL, NULL,
    &mmio_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    mmio_dt, NULL, NULL, NULL, NULL, NULL, &mmio_description
};

/* Reset routine */
static t_stat mmio_reset(DEVICE *dptr)
{
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &mmiodev, "mmiodev", TRUE);
    } else {
        /* Connect SS1 at base address */
        if(sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &mmiodev, "mmiodev", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n", __FUNCTION__, pnp->io_base);
            return SCPE_ARG;
        } else {
            DBG_PRINT(("SS1: Mapped I/O resource at 0x%04x, len=%d\n", pnp->io_base, pnp->io_size));
            mmio_unit[0].u4 = 0;
            mmio_unit[1].u4 = 1;
            mmio_unit[2].u4 = 2;
            mmio_unit[3].u4 = 3;
            mmio_pic[MASTER_PIC].IMR = 0xFF;
            mmio_pic[SLAVE_PIC].IMR = 0xFF;
        }
    }
    return SCPE_OK;
}

static int32 mmiodev(const int32 port, const int32 io, const int32 data)
{
    DBG_PRINT(("SS1: IO %s, Port %02x\n", io ? "WR" : "RD", port));
    if(io) {
        SS1_Write(port, data);
        return 0;
    } else {
        return(SS1_Read(port));
    }
}

#define SS1_M8259_L     0x00
#define SS1_M8259_H     0x01
#define SS1_S8259_L     0x02
#define SS1_S8259_H     0x03
#define SS1_8253_TC0    0x04
#define SS1_8253_TC1    0x05
#define SS1_8253_TC2    0x06
#define SS1_8253_CTL    0x07
#define SS1_9511A_DATA  0x08
#define SS1_9511A_CMD   0x09
#define SS1_RTC_CMD     0x0A
#define SS1_RTC_DATA    0x0B
#define SS1_UART_DATA   0x0C
#define SS1_UART_STAT   0x0D
#define SS1_UART_MODE   0x0E
#define SS1_UART_CMD    0x0F

extern int32 sio0d(const int32 port, const int32 io, const int32 data);
extern int32 sio0s(const int32 port, const int32 io, const int32 data);

static struct tm currentTime;

static uint8 SS1_Read(const uint32 Addr)
{
    uint8 cData = 0x00;

    uint8 sel_pic = MASTER_PIC;
    uint8 sel_tc = 0;
    time_t now;

    switch(Addr & 0x0F) {
        case SS1_S8259_L:
            sel_pic = SLAVE_PIC;
            /* fall through */
        case SS1_M8259_L:
            if((mmio_pic[sel_pic].OCW3 & 0x03) == 0x03) {
                cData = mmio_pic[sel_pic].ISR;
                sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                          " RD: %s PIC ISR=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
            } else if((mmio_pic[sel_pic].OCW3 & 0x03) == 0x02) {
                cData = mmio_pic[sel_pic].IRR;
                sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                          " RD: %s PIC IRR=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
            } else {
                cData = 0xFF;
            }
            break;
        case SS1_S8259_H:
            sel_pic = SLAVE_PIC;
            /* fall through */
        case SS1_M8259_H:
            cData = mmio_pic[sel_pic].IMR;
            sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: %s PIC IMR=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
            mmio_pic[sel_pic].IMR = cData;
            break;
        case SS1_8253_CTL:
            cData = mmio_tc[0].CTL;
            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: TC CTL=0x%02x.\n", PCX, cData);
            break;
        case SS1_8253_TC2:
            sel_tc++;
        case SS1_8253_TC1:
            sel_tc++;
        case SS1_8253_TC0:
            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: TC [%d]=0x%02x.\n", PCX, sel_tc, cData);
            break;
        case SS1_9511A_DATA:
        case SS1_9511A_CMD:
            sim_debug(MATH_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: Math Coprocessor not Implemented.\n", PCX);
            break;
        case SS1_RTC_CMD:
            cData = 0xFF;
            sim_debug(RTC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: RTC  Cmd=0x%02x.\n", PCX, cData);
            break;
        case SS1_RTC_DATA:
            sim_get_time(&now);
            now += mmio_rtc[0].clockDelta;
            currentTime = *localtime(&now);

            switch(mmio_rtc[0].digit_sel) {
            case 0:
                cData = currentTime.tm_sec % 10;
                break;
            case 1:
                cData = currentTime.tm_sec / 10;
                break;
            case 2:
                cData = currentTime.tm_min % 10;
                break;
            case 3:
                cData = currentTime.tm_min / 10;
                break;
            case 4:
                cData = currentTime.tm_hour % 10;
                break;
            case 5:
                cData = currentTime.tm_hour / 10;
                cData |= 0x08;  /* Set to 24-hour format */
                break;
            case 6:
                cData = currentTime.tm_wday;
                break;
            case 7:
                cData = currentTime.tm_mday % 10;
                break;
            case 8:
                cData = currentTime.tm_mday / 10;
                break;
            case 9:
                cData = (currentTime.tm_mon + 1) % 10;
                break;
            case 10:
                cData = (currentTime.tm_mon + 1) / 10;
                break;
            case 11:
                cData = currentTime.tm_year % 10;
                break;
            case 12:
                cData = (currentTime.tm_year % 100) / 10;
                break;
            default:
                cData = 0;
                break;
            }
            sim_debug(RTC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: RTC Data[%x]=0x%02x.\n", PCX, mmio_rtc[0].digit_sel, cData);

            break;
        case SS1_UART_DATA:
            cData = sio0d(Addr, 0, 0);
            sim_debug(UART_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: UART Data=0x%02x.\n", PCX, cData);
            break;
        case SS1_UART_STAT:
            cData = sio0s(Addr, 0, 0);
            sim_debug(UART_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: UART Stat=0x%02x.\n", PCX, cData);
            break;
        case SS1_UART_MODE:
        case SS1_UART_CMD:
            sim_debug(UART_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " RD: UART not Implemented.\n", PCX);
            break;
    }

    return (cData);

}

uint16 newcount = 0;
uint8 bc;

/* setClockSS1 sets the new ClockSS1Delta based on the provided digits */
static void setClockSS1(void) {
    struct tm newTime;
    time_t newTime_t;
    int32 year = 10 * mmio_rtc[0].digits[RTS_YEARS_10_DIGIT] + mmio_rtc[0].digits[RTS_YEARS_1_DIGIT];
    newTime.tm_year = year < 50 ? year + 100 : year;
    newTime.tm_mon  = 10 * mmio_rtc[0].digits[RTS_MONTHS_10_DIGIT] + mmio_rtc[0].digits[RTS_MONTHS_1_DIGIT] - 1;
    newTime.tm_mday = 10 * (mmio_rtc[0].digits[RTS_DAYS_10_DIGIT] & 3) + mmio_rtc[0].digits[RTS_DAYS_1_DIGIT]; // remove leap year information in days 10 digit
    newTime.tm_hour = 10 * (mmio_rtc[0].digits[RTS_HOURS_10_DIGIT] & 3) + mmio_rtc[0].digits[RTS_HOURS_1_DIGIT]; // also remove AM/PM- and 12/24-information in hours 10 digit
    newTime.tm_min  = 10 * mmio_rtc[0].digits[RTS_MINUTES_10_DIGIT] + mmio_rtc[0].digits[RTS_MINUTES_1_DIGIT];
    newTime.tm_sec  = 10 * mmio_rtc[0].digits[RTS_SECONDS_10_DIGIT] + mmio_rtc[0].digits[RTS_SECONDS_1_DIGIT];
    newTime.tm_isdst = -1;
    newTime_t = mktime(&newTime);
    if (newTime_t != (time_t)-1)
        mmio_rtc[0].clockDelta = (int32)(newTime_t - sim_get_time(NULL));
}

static void generate_mmio_interrupt(void);

static uint8 SS1_Write(const uint32 Addr, uint8 cData)
{

    uint8 sel_pic = MASTER_PIC;
    uint8 sel_tc = 0;
    uint8 sel_timer = 0;

    switch(Addr & 0x0F) {
        case SS1_S8259_L:
            sel_pic = SLAVE_PIC;    /* intentional falltrough */
        case SS1_M8259_L:
            if(cData & 0x10) {
                sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                          " WR: %s PIC ICW1=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
                mmio_pic[sel_pic].ICW[1] = cData;
                mmio_pic[sel_pic].config_cnt=1;
            } else {
                if(cData & 0x08) { /* OCW3 */
                    sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                              " WR: %s PIC OCW3=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
                    mmio_pic[sel_pic].OCW3 = cData;
                } else { /* OCW2 */
                    sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                              " WR: %s PIC OCW2=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
                    mmio_pic[sel_pic].OCW2 = cData;
                }
            }
            break;
        case SS1_S8259_H:
            sel_pic = SLAVE_PIC;
            /* fall through */
        case SS1_M8259_H:
            if(mmio_pic[sel_pic].config_cnt == 0) {
                sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT " WR: %s PIC IMR=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), cData);
                mmio_pic[sel_pic].IMR = cData;
                generate_mmio_interrupt();
            } else {
                mmio_pic[sel_pic].config_cnt++;
                sim_debug(PIC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT " WR: %s PIC ICW%d=0x%02x.\n", PCX, (sel_pic ? "Slave " : "Master"), mmio_pic[sel_pic].config_cnt, cData);
                mmio_pic[sel_pic].ICW[mmio_pic[sel_pic].config_cnt] = cData;

                mmio_unit[0].u3 = mmio_pic[SLAVE_PIC].ICW[2]+TC0_IRQ_OFFSET;
                mmio_unit[1].u3 = mmio_pic[SLAVE_PIC].ICW[2]+TC1_IRQ_OFFSET;
                mmio_unit[2].u3 = mmio_pic[SLAVE_PIC].ICW[2]+TC2_IRQ_OFFSET;

                if(mmio_pic[sel_pic].config_cnt == 4) {
                    mmio_pic[sel_pic].config_cnt = 0;
                }
            }
            break;
        case SS1_8253_CTL:
            mmio_tc[0].CTL = cData;
            sel_timer = (mmio_tc[0].CTL & I8253_CTL_SC_MASK) >> 6;
            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: TC CTL=0x%02x.\n",
                      PCX, mmio_tc[0].CTL);
            if(mmio_tc[0].CTL & I8253_CTL_BCD) {
                sim_debug(ERROR_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                          " Timer %d: BCD Mode not supported: TC CTL=0x%02x.\n",
                          PCX, sel_timer, mmio_tc[0].CTL);
            }
            mmio_tc[0].bcd[sel_timer] = (mmio_tc[0].CTL & I8253_CTL_BCD);
            mmio_tc[0].mode[sel_timer] = (mmio_tc[0].CTL & I8253_CTL_MODE_MASK) >> 1;
            mmio_tc[0].rl[sel_timer] = (mmio_tc[0].CTL & I8253_CTL_RL_MASK) >> 4;
            sim_debug(TRACE_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " Timer %d: Mode: %d, RL=%d, %s.\n",
                      PCX, sel_timer, mmio_tc[0].mode[sel_timer],
                      mmio_tc[0].rl[sel_timer],
                      mmio_tc[0].bcd[sel_timer] ? "BCD" : "Binary");
            newcount = 0;
            bc=0;
            break;
        case SS1_8253_TC2:
            sel_tc++;
        case SS1_8253_TC1:
            sel_tc++;
        case SS1_8253_TC0:
            if(mmio_tc[0].rl[sel_timer] == 3) {
                if(bc==0) {
                    newcount = cData;
                }
                if(bc==1) {
                    newcount |= (cData << 8);
                    sim_activate(&mmio_unit[sel_tc], newcount);
                }
                bc++;
            }

            if(mmio_tc[0].rl[sel_timer] == 2) {
                newcount = (cData << 8);
                sim_activate(&mmio_unit[sel_tc], newcount);
            }

            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: TC [%d]=0x%02x.\n", PCX, sel_tc, cData);
            if(sel_tc == 0) {
            }
            break;
        case SS1_9511A_DATA:
        case SS1_9511A_CMD:
            sim_debug(TRACE_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: Math Coprocessor not Implemented.\n", PCX);
            break;
        case SS1_RTC_CMD:
            mmio_rtc[0].digit_sel = cData & 0x0F;
            mmio_rtc[0].flags = (cData >> 4) & 0x0F;
            sim_debug(RTC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: RTC  Cmd=0x%02x (%s%s%s SEL=%x)\n",
                      PCX, cData,
                      mmio_rtc[0].flags & 0x4 ? "HOLD"   :"",
                      mmio_rtc[0].flags & 0x2 ? "WR"     :"",
                      mmio_rtc[0].flags & 0x1 ? "RD"     :"",
                      mmio_rtc[0].digit_sel);
            if (cData == 0) // set clock delta
                setClockSS1();
            break;
        case SS1_RTC_DATA:
            sim_debug(RTC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: RTC Data=0x%02x\n", PCX, cData);
            if (mmio_rtc[0].digit_sel <= RTS_YEARS_10_DIGIT)
                mmio_rtc[0].digits[mmio_rtc[0].digit_sel] = cData;
            break;
        case SS1_UART_DATA:
            sim_debug(UART_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: UART Data=0x%02x.\n", PCX, cData);
            sio0d(Addr, 1, cData);
            break;
        case SS1_UART_STAT:
            sim_debug(UART_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: UART Stat=0x%02x.\n", PCX, cData);
            sio0s(Addr, 1, cData);
            break;
        case SS1_UART_MODE:
        case SS1_UART_CMD:
            sim_debug(TRACE_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT
                      " WR: UART not Implemented.\n", PCX);
            break;
    }

    return(0);
}

void raise_mmio_interrupt(uint8 isr_index)
{
    uint8 irq_bit;
    if(isr_index < 7) { /* VI0-6 on master PIC */
        irq_bit = (1 << isr_index);
        mmio_pic[MASTER_PIC].ISR |= irq_bit;
        generate_mmio_interrupt();
    } else { /* VI7 is on slave PIC */
        mmio_pic[SLAVE_PIC].ISR |= 1;
        generate_mmio_interrupt();
    }
}
extern void cpu_raise_interrupt(uint32 irq);

static void generate_mmio_interrupt(void)
{
    uint8 irq, irq_pend, irq_index = 0, irq_bit = 0;

    uint8 pic;

    for(pic=MASTER_PIC;pic<=SLAVE_PIC;pic++) {
        irq_pend = (~mmio_pic[pic].IMR) & mmio_pic[pic].ISR;

        while(irq_pend) {

            irq_bit = irq_pend & 1;
            if(irq_bit) {
                mmio_pic[pic].IRR |= (irq_bit << irq_index);
                irq = mmio_pic[pic].ICW[2]+irq_index;
                sim_debug(IRQ_MSG, &mmio_dev, "Handling interrupt on %s PIC: IMR=0x%02x, ISR=0x%02x, IRR=0x%02x, index=%d\n", pic ? "SLAVE" : "MASTER", mmio_pic[pic].IMR, mmio_pic[pic].ISR, mmio_pic[pic].IRR, irq_index);
                cpu_raise_interrupt(irq);
                mmio_pic[pic].IRR &= ~(irq_bit << irq_index);
                mmio_pic[pic].ISR &= ~(irq_bit << irq_index);
                if(irq_pend & 0x7E) {
/*              sim_debug(IRQ_MSG, &mmio_dev, "Requeue interrupt on %s PIC: IMR=0x%02x, ISR=0x%02x, IRR=0x%02x, index=%d\n", pic ? "SLAVE" : "MASTER", mmio_pic[pic].IMR, mmio_pic[pic].ISR, mmio_pic[pic].IRR, irq_index);
*/
                    sim_activate(&mmio_unit[3], 1000);  /* requeue, because more interrupts are pending. */
                }
                break;
            } else {
                irq_index++;
                irq_pend = irq_pend >> 1;
            }
        }
    }
}


/* Unit service routine */
/* Unit 0-2 = Timer0-2, Unit3=ISR queue */
static t_stat mmio_svc (UNIT *uptr)
{
    uint8 cData;
    uint8 irq_bit = 0;

    /* Handle SS1 UART Rx interrupts here. */
    cData = sio0s(0x5D, 0, 0);
    if(cData & 2) { /* && ((mmio_pic[SLAVE_PIC].IMR & 0x80) == 0)) { */
        mmio_pic[SLAVE_PIC].ISR |= 0x80;
        generate_mmio_interrupt();
        sim_activate(uptr, 1000);  /* requeue, because we still need to handle the timer interrupt. */
    } else if((cData & 1) && ((mmio_pic[SLAVE_PIC].IMR & 0x40) == 0)) {
        sim_debug(IRQ_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT " Calling UART Tx ISR.\n", PCX);
        mmio_pic[SLAVE_PIC].ISR |= 0x40;
        generate_mmio_interrupt();
        sim_activate(uptr, 1000);   /* requeue, because we still need to handle the timer interrupt. */
    } else if (uptr->u4 == 0x3) {   /* ISR was requeued because additional interrupts were pending. */
        generate_mmio_interrupt();
    } else {
        switch(uptr->u4) {
            case 0:
                irq_bit = 2;
                break;
            case 1:
                irq_bit = 4;
                break;
            case 2:
                irq_bit = 8;
                break;
        }
        if(mmio_tc[0].mode[uptr->u4] == 0x0) {
            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT " Calling Timer%d ISR.\n", PCX, uptr->u4);
            mmio_pic[SLAVE_PIC].ISR |= irq_bit;
            generate_mmio_interrupt();
        }
        if(mmio_tc[0].mode[uptr->u4] == 0x3) {
            sim_debug(TC_MSG, &mmio_dev, "SS1: " ADDRESS_FORMAT " Calling Timer%d ISR.\n", PCX, uptr->u4);
            mmio_pic[SLAVE_PIC].ISR |= irq_bit;
            generate_mmio_interrupt();
            sim_debug(TC_MSG, &mmio_dev, "Timer %d, mode %d, reloading\n", uptr->u4, mmio_tc[0].mode[uptr->u4]);
                sim_activate(uptr, 33280);
        }
    }

    sim_activate(&mmio_unit[3], 1000000);  // requeue, because more interrupts are pending.

    return SCPE_OK;
}

