#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "network_controller.h"

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fsl_device_registers.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "spi_proto.h"
#include "spi_proto_slave.h"
#include "spi_edma_task.h"

#define max_PRIORITY (configMAX_PRIORITIES-1)

/* Use regular semaphore and one unguarded shared memory segment. Don't go fast.
 */
uint8_t msg[SPI_MSG_PAYLOAD_LEN] = {0};
SemaphoreHandle_t bcm_sem; //is there new data in the buffer?
void
spi_callback(struct spi_packet *p)
{
  if (!p->msg[0]) return;
  memcpy(msg, p->msg, SPI_MSG_PAYLOAD_LEN);
  xSemaphoreGive(bcm_sem); // TODO check for error
}

enum optype {
  READ = 1,
  WRITE = 2
};

#define IX_RW 0
#define IX_PAGE 1
#define IX_OFFSET 2
#define IX_LENGTH 3
#define IX_DATA_START 4
void
bcm_ctl_task(void *pvParams)
{
  //TODO do any init this task needs
  bcm_init_spi();

  uint8_t buf[256]; /* just use a big one and forget about it */
  uint8_t read_response[6] = {0xde, 0xad, 0xbe, 0xef};
  for (;;) {
    int gotsem = 0;
    while (!gotsem) {
      gotsem = pdTRUE == xSemaphoreTake(bcm_sem, 100); // TODO check for error
    }

    int ret;
    switch (msg[IX_RW]) {
    case READ:
      ret = normal_read_operation(msg[IX_PAGE], msg[IX_OFFSET],
                                      buf, msg[IX_LENGTH]);
      //TODO do any error handling
      slave_send_message(spi_proto::p, buf, msg[IX_LENGTH]);
      break;
    case WRITE:
      ret = normal_write_operation(msg[IX_PAGE], msg[IX_OFFSET],
                                       &msg[IX_DATA_START], msg[IX_LENGTH]);
      //TODO do any error handling
      slave_send_message(spi_proto::p, read_response, 6);
      break;
    default:
      //take no action. possibly could return a "didn't understand" message
      //TODO return a "bad command" message?
      continue;
    }
  }
}

void pins_init(void)
{
    CLOCK_EnableClock(kCLOCK_PortD);
    /* SPI on D0-D3 */
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt2);

}

int main(void)
{
  pins_init();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  bcm_sem = xSemaphoreCreateBinary();
  assert(bcm_sem != NULL); // "Couldn't create a semaphore"

  // we start the bcm_ctl_task before the spi comm task so that the semaphore is
  // taken from before it's given the first time.
  BaseType_t ret;
  ret = xTaskCreate(bcm_ctl_task, "bcm_ctl_task",
                    configMINIMAL_STACK_SIZE+100, NULL, max_PRIORITY-1, NULL);
  assert(ret != errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY);

  ret = xTaskCreate(spi_edma_task, "spi edma task",
                    configMINIMAL_STACK_SIZE+200, (void*) spi_callback,
                    max_PRIORITY, NULL);
  assert(ret != errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY);

  vTaskStartScheduler();

  for (;;) {
    __asm("NOP"); // can put a breakpoint to see if scheduler ever returns
  }
}
