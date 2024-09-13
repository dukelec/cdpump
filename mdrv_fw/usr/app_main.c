/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "math.h"
#include "app_main.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;

int CDCTL_SYS_CLK = 150000000; // 150MHz for cdctl01a

gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };

static gpio_t valve0 = { .group = VALVE0_GPIO_Port, .num = VALVE0_Pin };
static gpio_t valve1 = { .group = VALVE1_GPIO_Port, .num = VALVE1_Pin };
static gpio_t valve2 = { .group = VALVE2_GPIO_Port, .num = VALVE2_Pin };
static i2c_t sen_dev = { .hi2c = &hi2c2, .dev_addr = 0xda };

uart_t debug_uart = { .huart = &huart1 };

static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_cs };

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

static cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};             // CDNET


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head);

    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&packet_free_head, &packet_alloc[i].node);

    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, NULL, &r_int);

    if (r_dev.version >= 0x10) {
        // 16MHz / (2 + 2) * (73 + 2) / 2^1 = 150MHz
        cdctl_write_reg(&r_dev, REG_PLL_N, 0x2);
        d_info("pll_n: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_N));
        cdctl_write_reg(&r_dev, REG_PLL_ML, 0x49); // 0x49: 73
        d_info("pll_ml: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_ML));

        d_info("pll_ctrl: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_CTRL));
        cdctl_write_reg(&r_dev, REG_PLL_CTRL, 0x10); // enable pll
        d_info("clk_status: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        cdctl_write_reg(&r_dev, REG_CLK_CTRL, 0x01); // select pll

        d_info("clk_status after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        d_info("version after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_VERSION));
    } else {
        d_info("fallback to cdctl-b1 module, ver: %02x\n", r_dev.version);
        CDCTL_SYS_CLK = 40000000; // 40MHz
        cdctl_set_baud_rate(&r_dev, csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
    }

    cdn_add_intf(&dft_ns, &r_dev.cd_dev, csa.bus_net, csa.bus_cfg.mac);
}


extern uint32_t end; // end of bss
#define STACK_CHECK_SKIP 0x200
#define STACK_CHECK_SIZE (64 + STACK_CHECK_SKIP)

static void stack_check_init(void)
{
    int i;
    printf("stack_check_init: skip: %p ~ %p, to %p\n",
            &end, &end + STACK_CHECK_SKIP, &end + STACK_CHECK_SIZE);
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4)
        *(uint32_t *)(&end + i) = 0xababcdcd;
}

static void stack_check(void)
{
    int i;
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4) {
        if (*(uint32_t *)(&end + i) != 0xababcdcd) {
            printf("stack overflow %p (skip: %p ~ %p): %08lx\n",
                    &end + i, &end, &end + STACK_CHECK_SKIP, *(uint32_t *)(&end + i));
            d_error("stack overflow %p (skip: %p ~ %p): %08lx\n",
                    &end + i, &end, &end + STACK_CHECK_SKIP, *(uint32_t *)(&end + i));
            while (true);
        }
    }
}

#if 0
static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        t_l = get_systick();

        d_debug("ctl: state %d, t_len %d, r_len %d, irq %d\n",
                r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len,
                !gpio_get_value(r_dev.int_n));
        d_debug("  r_cnt %d (lost %d, err %d, no-free %d), t_cnt %d (cd %d, err %d)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt,
                r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
    }
}
#endif


static void set_valve(bool v0, bool v1, bool v2)
{
    gpio_set_value(&valve0, v0);
    gpio_set_value(&valve1, v1);
    gpio_set_value(&valve2, v2);
    csa.cur_valve = v0 | (v1 << 1) | (v2 << 2);
}

static void set_pump(uint16_t val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clip(val, 0, 1023));
    csa.cur_pwm = val;
}


static void pump_routine(void)
{
    static float set_pressure_bk = 0.0f;
    bool has_change = fabsf(csa.set_pressure - set_pressure_bk) > 0.0001f;
    bool is_zero = fabsf(csa.set_pressure) <= 0.0001f;
    
    if (has_change) {
        d_debug("has_change, is_zero: %d\n", is_zero);
        if (is_zero) {
            d_debug("disable pump\n");
            set_pump(0);
            set_valve(false, false, false); // open
        } else if (csa.set_pressure > 0) {
            set_valve(true, false, true); // blow
        } else {
            set_valve(true, true, false); // suck
        }
    }
    
    if (!is_zero) {
        float pid_tgt = csa.set_pressure > 0 ? csa.set_pressure : -csa.set_pressure;
        float pid_in = csa.set_pressure > 0 ? csa.sen_pressure : -csa.sen_pressure;

        pid_f_set_target(&csa.pid_pressure, pid_tgt);
        float pid_out = pid_f_compute(&csa.pid_pressure, pid_in);
        set_pump(pid_out);
    } else {
        pid_f_reset(&csa.pid_pressure, 0, 0);
        pid_f_set_target(&csa.pid_pressure, 0);
    }
    
    set_pressure_bk = csa.set_pressure;
    raw_dbg(0);
    csa.loop_cnt++;
}


static void read_sensor(void)
{
    uint8_t sen_state = 0;
    i2c_mem_read(&sen_dev, 0x02, &sen_state, 1);
    if (!(sen_state & 1))
        return;
    
    uint8_t sen_buf[5];
    i2c_mem_read(&sen_dev, 0x06, sen_buf, 5);
    
    int32_t tmp = (sen_buf[0] << 16) | (sen_buf[1] << 8) | sen_buf[2];
    if (tmp >= 0x800000)
        csa.sen_pressure = (float)(tmp - 0x1000000) / 0x800000 * 125 - 12.5f;
    else
        csa.sen_pressure = (float)tmp / 0x800000 * 125 - 12.5f;
    
    tmp = (sen_buf[3] << 8) | sen_buf[4];
    if (tmp > 32768)
        csa.sen_temperature = (tmp - 65844) / 256.0f;
    else
        csa.sen_temperature = (tmp - 308) / 256.0f;
    
    pump_routine();
    
    //d_info("state: %02x, pressure: %d.%.3d, temperature: %d.%.3d\n",
    //        sen_state, P_3F(csa.pressure), P_3F(csa.temperature));
}


void app_main(void)
{
    gpio_set_value(&led_r, 1);
    gpio_set_value(&led_g, 1);
    printf("\nstart app_main (mdrv-step)...\n");
    stack_check_init();
    load_conf();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    common_service_init();
    raw_dbg_init();
    printf("conf (mdrv-step): %s\n", csa.conf_from ? "load from flash" : "use default");
    d_info("conf (mdrv-step): %s\n", csa.conf_from ? "load from flash" : "use default");
    d_info("\x1b[92mColor Test\x1b[0m and \x1b[93mAnother Color\x1b[0m...\n");
    csa_list_show();
    //app_motor_init();

    delay_systick(100);
    gpio_set_value(&led_r, 0);
    //uint32_t t_last = get_systick();
    
    __HAL_TIM_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    i2c_mem_write(&sen_dev, 0x30, (void *) "\x0b", 1);
    pid_f_init(&csa.pid_pressure, true);
    
    while (true) {
        //if (get_systick() - t_last > (gpio_get_value(&led_g) ? 400 : 600)) {
        //    t_last = get_systick();
        //    gpio_set_value(&led_g, !gpio_get_value(&led_g));
        //}
        stack_check();
        //dump_hw_status();
        read_sensor();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        raw_dbg_routine();
        
        debug_flush(false);
    }
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == r_int.num) {
        cdctl_int_isr(&r_dev);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    printf("spi error... [%08lx]\n", hspi->ErrorCode);
}
