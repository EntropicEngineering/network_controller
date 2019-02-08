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

uint8_t normal_read_command(uint8_t bcm_addr);
void normal_write_command(uint8_t bcm_addr, uint8_t val);
void normal_read_command_buf(uint8_t bcm_addr, uint8_t *res, size_t len);
void normal_read_command_step4(uint8_t bcm_addr);

//TODO
#define SPI_BAUDRATE 400000
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
  cfg.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
  cfg.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
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
//TODO (in progress) rather than passing spi base around just #define it somewhere
int
//normal_read_operation(SPI_Type *base, uint8_t page, uint8_t oset
//       , uint8_t *result, size_t len)
/* read len bytes from oset in page, stored in result
 */
normal_read_operation(uint8_t page, uint8_t oset
                      , uint8_t *result, size_t len)

{
  //page 102, BCM 53128 datasheet
  int spif_timeout = 0;
  const int spif_timeout_limit = 10;
  uint8_t spi_status;
 step1:
  spi_status = normal_read_command(0xfe);
  if (!(spi_status & SPIF)) {
    if (++spif_timeout < spif_timeout_limit) {
      goto step1;
    } else {
      goto err;
    }
  }
  //step2:
  normal_write_command(0xff, page);
  //step3:
  normal_read_command(0x12);
  //step4:
  //step4 needs a custom loop because it just keeps clocking
  normal_read_command_step4(0xfe);
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
    .isPcsContinuous = false, // try to fix failure to continue after cfg_end
                              // used first time, didn't work
    .whichCtar = NETWORK_SPI_CTAR,
    .whichPcs = NETWORK_SPI_PCS,
    .clearTransferCount = false, // for the first one, this is a transaction
    .isEndOfQueue = true,
};
/* normal_read_command performs a BCM53128 Normal Read Command.
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
  DSPI_MasterWriteDataBlocking(base, &cfg_start, 0x60);
  spi_read[0] = DSPI_ReadData(SPI2);
  DSPI_MasterWriteDataBlocking(base, &cfg_middle, bcm_addr);
  spi_read[1] = DSPI_ReadData(SPI2);
  DSPI_MasterWriteDataBlocking(base, &cfg_end, 0x00);
  spi_read[2] = DSPI_ReadData(SPI2);
  //TODO return a value
  uint8_t spi_status = spi_read[2];
  return spi_status;
}
/* Write val to bcm_addr on the broadcom
 */
void
normal_write_command(uint8_t bcm_addr, uint8_t val)
{
  //TODO write [0x61, bcm_addr, val] to spi
  //TODO could make only the last one blocking?
  DSPI_MasterWriteDataBlocking(base, &cfg_start, 0x61);
  DSPI_MasterWriteDataBlocking(base, &cfg_middle, bcm_addr);
  DSPI_MasterWriteDataBlocking(base, &cfg_end, val);
}
/* performs a Normal Read Command but can return more than one byte.
 */
void
normal_read_command_buf(uint8_t bcm_addr, uint8_t *res, size_t len)
{
  //TODO write [0x60, bcm_addr] to spi, then without losing chip select
  //TODO write len 0's to spi, return received data from that

  DSPI_MasterWriteDataBlocking(base, &cfg_start, 0x60);
  DSPI_MasterWriteDataBlocking(base, &cfg_middle, bcm_addr);
  for (unsigned int i = 0; i < len; i++) {
    dspi_command_data_config_t *cfg = i+1==len ? &cfg_end : &cfg_middle;
    DSPI_MasterWriteDataBlocking(base, cfg, bcm_addr);
    res[i] = DSPI_ReadData(SPI2);
  }
}

void
normal_read_command_step4(uint8_t bcm_addr)
{
  //TODO write [0x60, addr, 0x00] to spi
  //TODO let status = last received byte. if status & RACK return successfully
  //TODO if not status & RACK transmit another byte and repeat, up to timeout
  //TODO timeout is 20 bytes tried
  DSPI_MasterWriteDataBlocking(base, &cfg_start, 0x60);
  DSPI_MasterWriteDataBlocking(base, &cfg_middle, bcm_addr);
  uint8_t spi_status = 0;
  int ix = 0;
  #define STEP4_RCV_SIZE 20
  uint8_t step4_rcv[20];

  while (!(spi_status & RACK) && ix < STEP4_RCV_SIZE) {
    DSPI_MasterWriteDataBlocking(base, &cfg_middle, 0x0);
    spi_status = step4_rcv[ix] = DSPI_ReadData(SPI2);
    ix++;
  }
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
    uint8_t statbuf[2] = {0};
    normal_read_operation(0x10, 0x12, statbuf, 2);

    //Serial.printf("getting link status summary\n");
    //get mac address (see datasheet for which)
    //uint8_t macbuf[BCM_53128_STATUS_LAST_SOURCE_ADDRESS_PORT_2.len] = {0};
    uint8_t macbuf[6] = {0};
    normal_read_operation(0x01, 0x1c, macbuf, 6);

    //TODO pause to inspect values in gdb
  }
}
