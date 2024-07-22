#include <stdint.h>
#include <string.h>
#include "fatfs.h"
#include "ini.h"

#include "main.h"

extern char SDPath[4];

uint8_t chip[4] = {0};
uint8_t cdip[4] = {0};
static int eth_cfg_handler(void* user, const char* section, const char* name, const char* value)
{
  #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
  if (MATCH("ethernet", "host_ip")) 
  {
    sscanf( value, "[%hhu, %hhu, %hhu, %hhu]", &chip[0], &chip[1], &chip[2], &chip[3] );
  } 
  else if (MATCH("ethernet", "device_ip"))
  {
    sscanf( value, "[%hhu, %hhu, %hhu, %hhu]", &cdip[0], &cdip[1], &cdip[2], &cdip[3] );
  } 
  else 
  {
    return 0;  /* unknown section/name, error */
  }
  return 1;
}

uint8_t cmac[6] = {0};
static int wake_cfg_handler(void* user, const char* section, const char* name, const char* value)
{
  #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
  if (MATCH("wake-on-lan", "enabled")) 
  {
    if( strcmp(value, "true") == 0 )
    {
      uint8_t enable = 1;
    }
  } 
  else if (MATCH("wake-on-lan", "mac_address")) 
  {
    sscanf( value, "[%hhi, %hhi, %hhi, %hhi, %hhi, %hhi]", &cmac[0], &cmac[1], &cmac[2], &cmac[3], &cmac[4], &cmac[5] );
  } else {
      return 0;  /* unknown section/name, error */
  }
  return 1;
}

extern uint8_t MY_IP_ADDRESS[4];
extern uint8_t RMT_IP_ADDRESS[4];
extern uint8_t host_mac_addr[6];

void parse_sd_ini(void)
{
  FATFS fileSystem;
  FIL testFile;
  FRESULT res;
  if(f_mount(&fileSystem, SDPath, 1) == FR_OK)
  {
    res = f_open(&testFile, "ethernet.ini", FA_OPEN_EXISTING | FA_READ);
    ini_parse_stream((ini_reader)f_gets, &testFile, eth_cfg_handler, NULL);
    res = f_close(&testFile);
    
    memset (&testFile, 0, sizeof(testFile));
    
    res = f_open(&testFile, "wake-on-lan.ini", FA_OPEN_EXISTING | FA_READ);
    ini_parse_stream((ini_reader)f_gets, &testFile, wake_cfg_handler, NULL);
    res = f_close(&testFile);    
    
    f_mount(NULL, SDPath, 1);
  }  
  
  memcpy( RMT_IP_ADDRESS, chip, 4 );
  memcpy( MY_IP_ADDRESS, cdip, 4 );
  memcpy( host_mac_addr, cmac, 6 );
}