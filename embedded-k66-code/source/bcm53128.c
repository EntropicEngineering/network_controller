#include "board.h"
#include "stdint.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "network_controller.h"

typedef struct {uint8_t page; uint8_t reg; uint8_t len;} bcm_reg;
#define BCM_53128_PAGE_STATUS_REGISTERS 0x01
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_0 ((bcm_reg) {0x01, 0x10, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_1 ((bcm_reg) {0x01, 0x16, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_2 ((bcm_reg) {0x01, 0x1c, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_3 ((bcm_reg) {0x01, 0x22, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_4 ((bcm_reg) {0x01, 0x28, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_5 ((bcm_reg) {0x01, 0x2e, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_6 ((bcm_reg) {0x01, 0x34, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_7 ((bcm_reg) {0x01, 0x3a, 6})
#define BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_IMP ((bcm_reg) {0x01, 0x40, 6})
#define AS_LEN(a,b,c) c

#define BCM_LAST_MAC_PORT_3 ((bcm_reg) {0x01, 0x22, 6})

struct macaddr {uint8_t addr[6];};
int last_mac(unsigned int port, struct macaddr *m);
int last_mac(unsigned int port, struct macaddr *m)
{
  //TODO this fetches MSB first in the array. For normal order reverse it.
  if (port > 8u) {
    memset(m, 0, sizeof(struct macaddr));
    return -2;
  }
  return normal_read_operation(0x01, 0x10 + 6*port, m->addr, 6);
}

int
set_port_enables(int port, bool txdis, bool rxdis)
{
  if (port > 7) {
    return -2;
  }
  uint8_t port_ctl_status;
  int readret = normal_read_operation(0x00, port, &port_ctl_status, 1);
  //TODO abort if read failed
  port_ctl_status &= 0x00 | ((!!txdis) << 1) | (!!rxdis);
  return normal_write_operation(0x00, port, &port_ctl_status, 1);
}

void break_on_me(void) {}; /* debugging breakpoint */
int main(void)
{
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  //shelving this trick for now
  //int macbuf[bcm_mac_reg(AS_LEN)] = {0};

  bcm_init_spi();

  for (;;) {
    //main must not exit
    //get status
    //Serial.printf("getting link status summary\n");
    //get mac address (see datasheet for which)
    //uint8_t macbuf[BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_2.len] = {0};
    uint8_t statbuf[2] = {0};
    normal_read_operation(0x10, 0x12, statbuf, 2);

    struct macaddr macs[8] = {0};
    int rets[8] = {0};

    for (int i = 0; i < 8; i++) {
      rets[i] = last_mac(i, &macs[i]);
    }

    uint8_t link_status_summary = 0;
    normal_read_operation(0x01, 0x00, &link_status_summary, 1);

    uint8_t ndio_port = 0;
    normal_read_operation(0, 0x78, &ndio_port, 1);

    uint8_t writeme = 0x00; //0x4a;
    normal_write_operation(0x00, 0x58, &writeme, 1);
    //set_port_enables(0, 1, 1);

    uint8_t led_buf_1[2] = {0x01, 0xff};
    uint8_t led_buf_2[2] = {0x01, 0xff};
    normal_write_operation(0x00, 0x1a, led_buf_1, 2);
    normal_write_operation(0x00, 0x18, led_buf_2, 2);

    break_on_me(); /* breakpoint target */
  }
}
