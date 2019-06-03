#include <thread>
#include "spi_datagram.h"

void cli_task(void);
//TODO don't duplicate
enum optype {
  READ = 1,
  WRITE = 2
};

int main(int argc, char *argv[])
{
  std::thread datagram(datagram_task);
  std::thread cli(cli_task);

  //TODO loop or someting
  for (;;){}

}


void
_spi_callback(struct spi_packet *p){
  //TODO parse the mesage, see what kin dof command it was, print it out

  //example output: Port 1 MAC: aa bb cc ee ff 11

  for (int i = 0; i < SPI_MSG_PAYLOAD_LEN; i++) {
    printf("%02x ", p->msg[i]);
  }
  puts("");

}
void (*spi_callback)(struct spi_packet *p) = _spi_callback;

/* Message Format
 * [R/W] [PAGE] [OFFSET] [LENGTH]
 * [R] [PAGE] [OFFSET] [LENGTH]
 * [W] [PAGE] [OFFSET] [LENGTH] [DATA] [DATA....]
 */
void
cli_task(void)
{
  //TODO wait for a character to be entered. send the appropriate datagram to
  // the som.
  uint8_t b_READ_MAC_0[4] = {READ, 0x01, 0x10, 6};
  uint8_t b_LINK_STATUS_SUMMARY[4] = {READ, 0x01, 0x00, 2};
  uint8_t b_LINK_STATUS_CHANGE[4] = {READ, 0x01, 0x02, 2};

  for (;;) {
  switch (getc(stdin)) {
  case '0':
    send_message(b_READ_MAC_0, 4);
    break;
  case 's':
    send_message(b_LINK_STATUS_SUMMARY, 4);
    break;
  case 'c':
    send_message(b_LINK_STATUS_CHANGE, 4);
    break;
  }
  }
}
