/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cdnet_core.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"
#include "pid_f.h"

#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0105

#define FRAME_MAX           60
#define PACKET_MAX          60

#define LOOP_FREQ   (64000000 / 64 / 200) // 5 KHz


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;
    bool            _reserved_bl;   // keep_in_bl for bl
    bool            save_conf;

    uint8_t         bus_net;
    cdctl_cfg_t     bus_cfg;
    bool            dbg_en;
    uint8_t         _reserved0[6];
    #define         _end_common _reserved1
    uint8_t         _reserved1[26];

    uint8_t         dbg_raw_msk;
    uint8_t         dbg_raw_th;     // len threshold (+ 1 samples < pkt size)
    regr_t          dbg_raw[1][6];

    uint8_t         _reserved2[20];
    pid_f_t         pid_pressure;
    uint8_t         _reserved3[42];

    // end of flash
    #define         _end_save _reserved4
    uint8_t         _reserved4[20];
    
    float           set_pressure;
    uint8_t         _reserved5[20];
    
    float           ori_pressure;
    float           bias_pressure;
    uint8_t         _reserved6[12];
    
    float           sen_pressure;       // kpa
    float           sen_temperature;    // c
    
    uint8_t         _reserved7[32];
    uint8_t         cur_valve;
    uint16_t        cur_pwm;
    uint32_t        loop_cnt;

} csa_t; // config status area


typedef uint8_t (*hook_func_t)(uint16_t sub_offset, uint8_t len, uint8_t *dat);

typedef struct {
    regr_t          range;
    hook_func_t     before;
    hook_func_t     after;
} csa_hook_t;


extern csa_t csa;
extern const csa_t csa_dft;

extern regr_t csa_w_allow[]; // writable list
extern int csa_w_allow_num;

extern csa_hook_t csa_w_hook[];
extern int csa_w_hook_num;
extern csa_hook_t csa_r_hook[];
extern int csa_r_hook_num;

extern uint32_t end; // end of bss

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

void common_service_init(void);
void common_service_routine(void);

uint8_t motor_w_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint8_t ref_volt_w_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint8_t drv_mo_r_hook(uint16_t sub_offset, uint8_t len, uint8_t *dat);
void app_motor_routine(void);
void app_motor_init(void);
void raw_dbg(int idx);
void limit_det_isr(void);

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;
extern list_head_t frame_free_head;
extern cdctl_dev_t r_dev;

#endif
