// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//



/* Example pinmap for Bluepill I2Cs (by Testato)

 I2C-1 standard pins: PB7(sda) PB6(scl)
 Use it by "Wire" without pin declaration
  Wire.begin();

 I2C-1 alternative pins: PB9(sda) PB8(scl)
 Remap the first I2C before call begin()
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

 I2C-2: PB11(sda) PB10(scl)
 Remap the second I2C before call begin()
  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.begin();

 If you want to use the two I2Cs simultaneously, create a new instance for the second I2C
  TwoWire Wire2(PB11,PB10);
  Wire2.begin();
 
*/


#include <Wire.h>

#include "USBSerial.h"

#define Serial SerialUSB

USBSerial Serial;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  Wire.begin();
  Serial.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  for(int c = 0; c < 5; c++) {
    
    Serial.println("Scanning...");
  
    nDevices = 0;
    for(address = 1; address < 127; address++) {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
  
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) 
          Serial.print("0");
        Serial.println(address, HEX);
  
        nDevices++;
      }
      else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16) 
          Serial.print("0");
        Serial.println(address, HEX);
      }    
    }
    if (nDevices == 0) {
      Serial.println("No I2C devices found");
    }
    else {
      for(;nDevices>0;nDevices--) {
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED on by making the voltage LOW
        delay(150);              // wait for a second
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off (HIGH is the voltage level)
        delay(150);              // wait for a second
      }
      Serial.println("done");
      Serial.println("0aaaaaaaaa_1aaaaaaaaa_2aaaaaaaaa_3aaaaaaaaa_4aaaaaaaaa_5aaaaaaaaa_6aaaaaaaaa_7aaaaaaaaa_8aaaaaaaaa_9aaaaaaaaa_10aaaaaaaaa_11aaaaaaaaa_12aaaaaaaaa_13aaaaaaaaa_14aaaaaaaaa_15aaaaaaaaa_");
    }
    delay(5000);           // wait 5 seconds for next scan
  }
  Serial.println("Resetting to bootloader...");
  JumpToBootloader();

}

extern "C" void usbd_interface_init() {
  
}

void JumpToBootloader(void) {
  void (*SysMemBootJump)(void);

  /**
   * Step: Set system memory address. 
   *       
   *       For STM32F429, system memory is on 0x1FFF 0000
   *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
   */
  //volatile uint32_t addr = 0x1FFF0000;
  volatile uint32_t addr = 0x0;
  
  Serial.end();
  
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12,LOW);
  HAL_Delay(10);
  pinMode(PA12, INPUT);


  /**
   * Step: Disable RCC, set it to default (after reset) settings
   *       Internal clock, no PLL, etc.
   */
//#if defined(USE_HAL_DRIVER)
  HAL_RCC_DeInit();
//#endif /* defined(USE_HAL_DRIVER) */
//#if defined(USE_STDPERIPH_DRIVER)
  //RCC_DeInit();
//#endif /* defined(USE_STDPERIPH_DRIVER) */

  HAL_DeInit();
  
  /**
   * Step: Disable all interrupts
   */
  __disable_irq();
  
  /**
   * Step: Disable systick timer and reset it to default values
   */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /**
   * Step: Remap system memory to address 0x0000 0000 in address space
   *       For each family registers may be different. 
   *       Check reference manual for each family.
   *
   *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
   *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
   *       For others, check family reference manual
   */
  //Remap by hand... {
//#if defined(STM32F4)
  //SYSCFG->MEMRMP = 0x01;
//#endif
//#if defined(STM32F0)
  //SYSCFG->CFGR1 = 0x01;
//#endif
  //} ...or if you use HAL drivers
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH(); //Call HAL macro to do this for you
  
  /**
   * Step: Set jump memory location for system memory
   *       Use address with 4 bytes offset which specifies jump location where program starts
   */
  SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
  
  /**
   * Step: Set main stack pointer.
   *       This step must be done last otherwise local variables in this function
   *       don't have proper value since stack pointer is located on different position
   *
   *       Set direct address location which specifies stack pointer in SRAM location
   */
  __set_MSP(*(uint32_t *)(addr));
  
  /**
   * Step: Actually call our function to jump to set location
   *       This will start system memory execution
   */
  SysMemBootJump();
  
  /**
   * Step: Connect USB<->UART converter to dedicated USART pins and test
   *       and test with bootloader works with STM32 Flash Loader Demonstrator software
   */
}
