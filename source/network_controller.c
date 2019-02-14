/*
 * plan: use DSPI_MasterTransferBlocking with continuous PCS set. Read the
 * return value from the rxbuffer in the master spi state struct.
 */
#include "board.h"
#include "stdint.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "clock_config.h"

#define RACK (1u<<5)
#define SPIF (1u<<7)

//TODO should be possible to not set isEndOfQueue and then not have to set SR
// all the time

//TODO integrate this code with datagram API for minimal spi datagram -> switch
// port control example

uint8_t normal_read_command(uint8_t bcm_addr);
void normal_write_command(uint8_t bcm_addr, uint8_t val);
void normal_read_command_buf(uint8_t bcm_addr, uint8_t *res, size_t len);
int normal_read_command_step4(uint8_t bcm_addr);

#define SPI_BAUDRATE 24000000
SPI_Type *base = SPI2;
static dspi_master_config_t cfg;
//CTARs are duplicated per-spi, no clash possibility
#define NETWORK_SPI_CTAR kDSPI_Ctar0
//SPI2_PCS0 can be D11 or B20, here it's B20
#define NETWORK_SPI_PCS kDSPI_Pcs0
#define DSPI_MASTER_CLK_SRC DSPI2_CLK_SRC
void
bcm_init_spi()
{
  // SPI2
  CLOCK_EnableClock(kCLOCK_PortB);
  PORT_SetPinMux(PORTB, 20U, kPORT_MuxAlt2); /* SPI2_PCS0 */
  PORT_SetPinMux(PORTB, 21U, kPORT_MuxAlt2); /* SPI2_SCK  */
  PORT_SetPinMux(PORTB, 22U, kPORT_MuxAlt2); /* SPI2_SOUT */
  PORT_SetPinMux(PORTB, 23U, kPORT_MuxAlt2); /* SPI2_SIN  */

  cfg.whichCtar = NETWORK_SPI_CTAR;
  cfg.whichPcs = NETWORK_SPI_PCS;
  cfg.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
  cfg.ctarConfig.baudRate = SPI_BAUDRATE;
  cfg.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
  cfg.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
  cfg.ctarConfig.direction = kDSPI_MsbFirst;
  cfg.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / SPI_BAUDRATE;
  cfg.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / SPI_BAUDRATE;
  cfg.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / SPI_BAUDRATE;

  cfg.enableContinuousSCK = false;
  cfg.enableRxFifoOverWrite = false;
  cfg.enableModifiedTimingFormat = false;
  cfg.samplePoint = kDSPI_SckToSin0Clock;
  // above this line is unconfirmed!
  cfg.ctarConfig.bitsPerFrame = 8U;

  uint32_t srcClock_Hz; // TODO ensure this is 400khz
  srcClock_Hz = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
  DSPI_MasterInit(base, &cfg, srcClock_Hz);
}
#define DELAY_AMOUNT 300
void
delay_a_bit(void)
{
  for (int i = 0; i <DELAY_AMOUNT;i++)
     __asm("NOP");
}
int
/* read len bytes from oset in page, stored in result. Page 100 BCM53128
 * datasheet
 */
normal_read_operation(uint8_t page, uint8_t oset
                      , uint8_t *result, size_t len)
{
  int spif_timeout = 0;
  const int spif_timeout_limit = 10;
  uint8_t spi_status;
 step1:
  spi_status = normal_read_command(0xfe);
  if ((spi_status & SPIF)) {
    if (++spif_timeout < spif_timeout_limit) {
      delay_a_bit();
      goto step1;
    } else {
      normal_write_command(0xff, page);
      goto err;
    }
  }
  delay_a_bit();
  //step2:
  normal_write_command(0xff, page);
  delay_a_bit();
  //step3:
  normal_read_command(oset);
  delay_a_bit();
  //step4:
  //step4 needs a custom loop because it just keeps clocking
  int got_rack = normal_read_command_step4(0xfe);
  if (!got_rack) {
    normal_write_command(0xff, page);
    goto err;
  }
  delay_a_bit();
  //step5:
  normal_read_command_buf(0xf0, result, len);
  return 0;
 err:
  return -1;
}
//TODO initialize these correctly
dspi_command_data_config_t cfg_start = {
    .isPcsContinuous = true,
    .whichCtar = NETWORK_SPI_CTAR,
    .whichPcs = NETWORK_SPI_PCS,
    .clearTransferCount = true, // for the first one, this is a transaction
    .isEndOfQueue = false,
  };
dspi_command_data_config_t cfg_middle = {
    .isPcsContinuous = true,
    .whichCtar = NETWORK_SPI_CTAR,
    .whichPcs = NETWORK_SPI_PCS,
    .clearTransferCount = false, // for the first one, this is a transaction
    .isEndOfQueue = false,
  };

dspi_command_data_config_t cfg_end = {
    .isPcsContinuous = false,
    .whichCtar = NETWORK_SPI_CTAR,
    .whichPcs = NETWORK_SPI_PCS,
    .clearTransferCount = false, // for the first one, this is a transaction
    .isEndOfQueue = true, // TODO try removing this and all the base->SR writes
};
/* Write one byte to the given SPI, returning the response. Ensures the FIFO
 * doesn't get out of sync.
 */
static inline uint32_t
dspi_write(SPI_Type *base, dspi_command_data_config_t *cfg, uint16_t val)
{
  DSPI_MasterWriteDataBlocking(base, cfg, val);
  return DSPI_ReadData(base);
}
/* normal_read_command performs a BCM53128 Normal Read Command. Page 102 et al
 * Note that this is distinct from a normal read operation.
 */
uint8_t
normal_read_command(uint8_t bcm_addr)
{
  // write [0x60, bcm_addr, 0x00] to spi, return third received byte
  //TODO could make only the last one blocking
  uint8_t spi_read[3];
  // TODO we need this to clear EOQF, but we might only need to do that because
  // we set isEndOfQueue in cfg_end, can we not do that?
  base->SR = SPI_SR_EOQF_MASK;
  spi_read[0] = dspi_write(base, &cfg_start, 0x60);
  spi_read[1] = dspi_write(base, &cfg_middle, bcm_addr);
  spi_read[2] = dspi_write(base, &cfg_end, 0x00);

  return spi_read[2];
}
/* Write val to bcm_addr on the broadcom
 */
void
normal_write_command(uint8_t bcm_addr, uint8_t val)
{
  // write [0x61, bcm_addr, val] to spi
  // TODO we need this to clear EOQF, but we might only need to do that because
  // we set isEndOfQueue in cfg_end, can we not do that?
  base->SR = SPI_SR_EOQF_MASK;
  //TODO could make only the last one blocking?
  dspi_write(base, &cfg_start, 0x61);
  dspi_write(base, &cfg_middle, bcm_addr);
  dspi_write(base, &cfg_end, val);
}
/* performs a Normal Read Command but can return more than one byte.
 */
void
normal_read_command_buf(uint8_t bcm_addr, uint8_t *res, size_t len)
{
  // write [0x60, bcm_addr] to spi, then without losing chip select
  // write len 0's to spi, return received data from that

  // TODO we need this to clear EOQF, but we might only need to do that because
  // we set isEndOfQueue in cfg_end, can we not do that?
  base->SR = SPI_SR_EOQF_MASK;
  dspi_write(base, &cfg_start, 0x60);
  dspi_write(base, &cfg_middle, bcm_addr);
  for (unsigned int i = 0; i < len; i++) {
    dspi_command_data_config_t *cfg = i+1==len ? &cfg_end : &cfg_middle;
    res[i] = dspi_write(base, cfg, 0x00);
  }
}

int
normal_read_command_step4(uint8_t bcm_addr)
{
  // write [0x60, addr, 0x00] to spi
  // TODO we need this to clear EOQF, but we might only need to do that because
  // we set isEndOfQueue in cfg_end, can we not do that?

  base->SR = SPI_SR_EOQF_MASK;
  uint32_t sr_on_entry = base->SR;
  //TODO let status = last received byte. if status & RACK return successfully
  //TODO if not status & RACK transmit another byte and repeat, up to timeout
  //TODO timeout is 20 bytes tried
  int rcvpre[2] = {0};
  rcvpre[0] = dspi_write(base, &cfg_start, 0x60);
  rcvpre[1] = dspi_write(base, &cfg_middle, bcm_addr);
  uint8_t spi_status = 0;
  int ix = 0;
  #define STEP4_RCV_SIZE 20
  uint8_t step4_rcv[20] = {0};

  int retval = 0;
  do { // dowhile for debugging purposes
    spi_status = step4_rcv[ix] = dspi_write(base, &cfg_middle, 0x0);
    if (spi_status & RACK) {
      retval = 1;
      break;
    }
  } while (++ix < STEP4_RCV_SIZE);
  //clock dummy byte to reset Chip Select
  dspi_write(base, &cfg_end, 0x00);
  uint32_t sr_on_exit = base->SR;
  int RXDF = sr_on_exit & SPI_SR_RFDF_MASK;
  return retval;
}

int
normal_write_command_buf(int oset, uint8_t *buf, size_t len)
{
  // TODO we need this to clear EOQF, but we might only need to do that because
  // we set isEndOfQueue in cfg_end, can we not do that?
  base->SR = SPI_SR_EOQF_MASK;
  dspi_write(base, &cfg_start, 0x61);
  dspi_write(base, &cfg_middle, oset);
  for (unsigned int i = 0; i < len; i++) {
    dspi_command_data_config_t *cfg = i+1==len ? &cfg_end : &cfg_middle;
    dspi_write(base, cfg, buf[i]);
  }
}

/* Write the bytes in the buffer result to the bcm location oset in page.
 */
int
normal_write_operation(uint8_t page, uint8_t oset
                      , uint8_t *buf, size_t len)
{
  /* Normal Write operation consists of 3 transactions (three SS operations)
   * 1. Issue a Normal Read Command (opcode = 0x60) to poll the SPIF bit in the
   *    SPI Status register (0xFE) to determine the operation can start.
   * 2. Issue a Normal Write command (opcode = 0x61) to setup the accessed
   *    register page value into the page register (0xFF).
   * 3. Issue a Normal Write command (opcode = 0x61) to setup the accessed
   *    register address value, followed by the write content starting from a
   *    lower byte.
   */
  int spi_status_times = 0;
  const int spi_status_timeout = 20;
  int spi_status = 0;
 step1:
  spi_status = normal_read_command(0xfe);
  if (spi_status & SPIF) {
    if (spi_status_times++ > spi_status_timeout) {
      normal_write_command(0xff, page);
      return -1;
    }
    goto step1;
  }
  normal_write_command(0xff, page);
  int ret3 = normal_write_command_buf(oset, buf, len);
  return 0;
}

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
