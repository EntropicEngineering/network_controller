#include <SPI.h>
#define SPI_STATUS_REGISTER 0xfe
#define SPI_PAGE_REGISTER 0xff
#define SPI_DATA_IO_REGISTER 0xf0
#define NORMAL_READ_CMD 0x60
#define NORMAL_WRITE_CMD 0x61

#define SLAVE_SELECT 10

#define SPI_SS_SPIF (1u<<7)
#define SPI_SS_RACK (1u<<5)
#define SPI_SS_MDIO_Start (1u<<2)

//not confident in these settings
//MODE3 is correct
SPISettings settings(24*1000*1000, MSBFIRST, SPI_MODE3);

void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  Serial.begin(9600);

  pinMode(SLAVE_SELECT, OUTPUT);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
}

void loop() {
  Serial.printf("MAIN LOOP\n");
  // put your main code here, to run repeatedly:
  byte statbuf[2] = {0};
  normal_read_operation(0x10, 0x12, statbuf, 2);

  Serial.printf("getting link status summary\n");
  byte macbuf[6] = {0};
  normal_read_operation(0x01, 0x1c, macbuf, 6);
  Serial.printf("MAC:");
  for (int i = 0; i < 6; i++)
    Serial.printf(" %02x", macbuf[i]);
  Serial.printf("\n");

  Serial.printf("reading port 2 (3 on box) status\n");
  byte portbuf[1] = {0};
  normal_read_operation(0x00, 0x02, portbuf, 1);
  Serial.printf("PORT STATUS:");
  for (int i = 0; i < 1; i++)
    Serial.printf(" %02x", portbuf[i]);
  Serial.printf("\n");

  delay(1000);
}
//because ~SS must be low, write SS high
#define SS_POLARITY 0
void slave_select_on(void) {
  digitalWrite(SLAVE_SELECT, SS_POLARITY ? HIGH : LOW);
}
void slave_select_off(void) {
  digitalWrite(SLAVE_SELECT, SS_POLARITY ? LOW : HIGH);
}

void do_a_transfer(byte *rcv, byte *snd, size_t n)
{
  {
    SPI.beginTransaction(settings);
    slave_select_on();
    for (int i = 0; i < n; i++)
      rcv[i] = SPI.transfer(snd[i]);
    slave_select_off();
    SPI.endTransaction();
  }
}
/* 80 char spacer because arduino ide doesn't show line length
----+++++----+++++----+++++----+++++----+++++----+++++----+++++----+++++----+++++
*/
void normal_read_operation(byte page, byte offset, byte *buf, size_t len) {
  /*
   * Normal Read Operation
   * Normal Read operation consists of five transactions (five SS operations):
   * 1. Issue a Normal Read Command (opcode = 0x60) to poll the SPIF bit in the
   *    SPI Status register (0xFE) to determine the operation can start.
   * 2. Issue a Normal Write command (opcode = 0x61) to write the register page
   *    value into the SPI Page register 0xFF.
   * 3. Issue a Normal Read command (opcode = 0x60) to setup the required
   *    RoboSwitch register address.
   * 4. Issue a Normal Read command (opcode = 0x60) to poll the RACK bit in the
   *    SPI status register(0xFE) to determine the completion of read (register
   *    content gets loaded in SPI Data I/O register).
   * 5. Issue a Normal Read command (opcode = 0x60) to read the specific
   *    registers' content placed in the SPI Data I/O register (0xF0).
   */

  int first_time = 1;
  step_1:
  byte step1_snd[3] = {0x60, 0xfe, 0x00};
  byte step1_rcv[3] = {0};
  do_a_transfer(step1_rcv, step1_snd, 3);
  byte spi_status = step1_rcv[2];
  //if (first_time)
    Serial.printf("step1 SEND: %02x %02x %02x\n", step1_snd[0], step1_snd[1], step1_snd[2]);
    Serial.printf("step1 RECV: %02x %02x %02x\n", step1_rcv[0], step1_rcv[1], step1_rcv[2]);
    Serial.printf("SPI Status: SPIF=%c RACK=%c MDIO=%c\n"
      , step1_rcv[2]&SPI_SS_SPIF ? '1' : '0'
      , step1_rcv[2]&SPI_SS_RACK ? '1' : '0'
      , step1_rcv[2]&SPI_SS_MDIO_Start ? '1' : '0');

  if ((spi_status & SPI_SS_SPIF)) {
    if (first_time) {
      Serial.printf("delaying in step 1");
      first_time = 0;
    } else {
      Serial.printf(".");
    }
    delay(100);
    goto step_1;
  }
delay(10);
  step_2:
  byte step2_snd[3] = {0x61, 0xff, page};
  byte step2_rcv[3] = {0};
  do_a_transfer(step2_rcv, step2_snd, 3);

  Serial.printf("step2 SEND: %02x %02x %02x\n", step2_snd[0], step2_snd[1], step2_snd[2]);
  Serial.printf("step2 RECV: %02x %02x %02x\n", step2_rcv[0], step2_rcv[1], step2_rcv[2]);
delay(10);
  step_3:
  byte step3_snd[3] = {0x60, offset, 0x0};
  byte step3_rcv[3];
  SPI.beginTransaction(settings);
  slave_select_on();
  /*
  step3_rcv[0] = SPI.transfer(0x60);
  step3_rcv[1] = SPI.transfer(offset);
  step3_rcv[2] = SPI.transfer(0x0); // dummy byte
  */
  for (int i = 0; i < 3; i++) step3_rcv[i] = SPI.transfer(step3_snd[i]);
  slave_select_off();
  SPI.endTransaction();
  Serial.printf("step3 SEND: %02x %02x %02x\n", step3_snd[0], step3_snd[1], step3_snd[2]);
  Serial.printf("step3 RECV: %02x %02x %02x\n", step3_rcv[0], step3_rcv[1], step3_rcv[2]);
delay(10);
  step_4:
  #define STEP4_MAX_ATTEMPTS 20
  #define STEP4_RCV_SIZE (STEP4_MAX_ATTEMPTS+2)
  byte step4_rcv[STEP4_RCV_SIZE] = {0};
  SPI.beginTransaction(settings);
  slave_select_on();
  step4_rcv[0] = SPI.transfer(0x60);
  step4_rcv[1] = SPI.transfer(0xfe);
  int ix = 2;
  while (!(spi_status & SPI_SS_RACK) && ix < STEP4_RCV_SIZE)
  spi_status = step4_rcv[ix++] = SPI.transfer(0x0);
  //spi_status = step4_rcv[2] = SPI.transfer(0x0);
  //spi_status = step4_rcv[3] = SPI.transfer(0x0);
  //spi_status = step4_rcv[4] = SPI.transfer(0x0);
  slave_select_off();
  SPI.endTransaction();


  Serial.printf("step4:");
  for (int i = 0; i < STEP4_RCV_SIZE; i++) {
    Serial.printf(" %02x", step4_rcv[i]);
  }
  Serial.printf("\n");
  Serial.printf("SPI Status: SPIF=%c RACK=%c MDIO=%c\n"
      , spi_status&SPI_SS_SPIF ? '1' : '0'
      , spi_status&SPI_SS_RACK ? '1' : '0'
      , spi_status&SPI_SS_MDIO_Start ? '1' : '0');
delay(10);
  if (!(spi_status & SPI_SS_RACK)) {
    //delay(10);
    //Serial.printf("delaying in step 4!\n");
    //goto step_4;

    Serial.printf("step 4 timed out (%d attempts)\n", STEP4_MAX_ATTEMPTS);
    //timeout, so we repeat step2 and then return
    byte step2_rcv[3];
    SPI.beginTransaction(settings);
    slave_select_on();
    step2_rcv[0] = SPI.transfer(0x61);
    step2_rcv[1] = SPI.transfer(0xff);
    step2_rcv[2] = SPI.transfer(page);
    slave_select_off();
    SPI.endTransaction();
    return;
  }
delay(10);
  step_5:
  byte ignore;
  SPI.beginTransaction(settings);
  slave_select_on();
  ignore = SPI.transfer(0x60);
  ignore = SPI.transfer(0xf0);
  byte data[16] = {0};
  for (int i = 0; i < len; i++)
    data[i] = SPI.transfer(0x0);
  //data[1] = SPI.transfer(0x0);
  //data[2] = SPI.transfer(0x0);
  //data[3] = SPI.transfer(0x0);
  slave_select_off();
  SPI.endTransaction();

  memcpy(buf, data, len);

  Serial.printf("data is %02x %02x X> %02x %02x\n", data[0], data[1], data[2], data[3]);
}

