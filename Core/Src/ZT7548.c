#include "ZT7548.h"
#include "main.h"
#include "i2c.h"

uint8_t cmd_buf[10];
uint16_t Chip_ID;
uint16_t CHECKSUM;

void ZT7548_init()
{

//  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
//  LL_mDelay(2);
#ifdef HW_i2C
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CHECKSUM_RESULT, read_buf, 2);
  //CHECKSUM = read_buf[1] << 8 | read_buf[0];
  //Serial2.printf("ZT7538_CHECKSUM_RESULT = 0x%X\n", CHECKSUM);
  //if(CHECKSUM != 0x55AA)
  //{
  //    Serial2.println("Do firmware upgrade");
  //}
  cmd_buf[0] = 0x01;
  cmd_buf[1] = 0x00;
  I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC000, cmd_buf, 2);

  I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0xCC00, read_buf, 2);
  Chip_ID = read_buf[1] << 8 | read_buf[0];
  #ifdef ENABLE_LOGGING
  Serial2.printf("chip code = 0x%X\n", Chip_ID);
  #endif
  I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC004, NULL, NULL);

  cmd_buf[0] = 0x01;
  cmd_buf[1] = 0x00;
  I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC002, cmd_buf, 2);
  LL_mDelay(2);

  cmd_buf[0] = 0x01;
  cmd_buf[1] = 0x00;
  I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC001, cmd_buf, 2);
  LL_mDelay(FIRMWARE_ON_DELAY);

  //cmd_buf[0] = 0x0A;
  //cmd_buf[1] = 0x00;
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0x000A, cmd_buf, 2);

  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0011, read_buf, 1);
  //Serial2.printf("ic_name = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001C, read_buf, 1);
  //Serial2.printf("ic_vendor_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0014, read_buf, 1);
  //Serial2.printf("hw_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001E, read_buf, 1);
  //Serial2.printf("tsm_module_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0012, read_buf, 1);
  //Serial2.printf("major_fw_version = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0121, read_buf, 1);
  //Serial2.printf("minor_fw_version = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0013, read_buf, 1);
  //Serial2.printf("release_version = 0x%X\n", read_buf[0]);

  //cmd_buf[0] = 0x00;
  //cmd_buf[1] = 0x00;
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_TOUCH_MODE, cmd_buf, 2);
  //delay(1);
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CLEAR_INT_STATUS_CMD, NULL, NULL);

#endif
#ifdef Soft_i2C
  ZT7548.begin();

  cmd_buf[0] = 0x00;
  cmd_buf[1] = 0xC0;
  cmd_buf[2] = 0x01;
  cmd_buf[3] = 0x00;
  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR);
  ZT7548.write(cmd_buf, 4);
  ZT7548.endTransmission();
  delay(10);

  cmd_buf[0] = 0x02;
  cmd_buf[1] = 0xC0;
  cmd_buf[2] = 0x01;
  cmd_buf[3] = 0x00;
  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR); 
  ZT7548.write(cmd_buf, 4);
  ZT7548.endTransmission(); 
  delay(10);


  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR);
  ZT7548.write(0x04);
  ZT7548.write(0xC0);
  ZT7548.endTransmission();
  delay(10);

  cmd_buf[0] = 0x01;
  cmd_buf[1] = 0xC0;
  cmd_buf[2] = 0x01;
  cmd_buf[3] = 0x00;
  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR);
  ZT7548.write(cmd_buf, 4);
  ZT7548.endTransmission(); 
  delay(10);
#endif
#ifdef ENABLE_LOGGING
Serial2.println("ZT7548 initial succeed");
#endif

}
