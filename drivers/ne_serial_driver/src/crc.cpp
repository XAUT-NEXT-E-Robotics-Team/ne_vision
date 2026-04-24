/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-22 23:35:41
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-03-22 23:35:50
 * @FilePath: /ne_vision/ne_serial/src/crc.cpp
 * @Description: 我永远喜欢雪之下雪乃
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#include "ne_serial_driver/crc.hpp"

namespace ne_io
{

unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage,
                                 unsigned int   dwLength,
                                 unsigned char  ucCRC8)
{
  unsigned char ucIndex;
  while (dwLength--)
  {
    ucIndex = ucCRC8 ^ (*pchMessage++);
    ucCRC8 = CRC8_TAB[ucIndex];
  }
  return ucCRC8;
}

unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage,
                                   unsigned int   dwLength)
{
  unsigned char ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return 0;
  ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
  return (ucExpected == pchMessage[dwLength - 1]);
}

void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength)
{
  unsigned char ucCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return;
  ucCRC =
      Get_CRC8_Check_Sum((unsigned char*)pchMessage, dwLength - 1, CRC8_INIT);
  pchMessage[dwLength - 1] = ucCRC;
}

uint16_t
Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t chData;
  if (pchMessage == nullptr)
  {
    return 0xFFFF;
  }
  while (dwLength--)
  {
    chData = *pchMessage++;
    (wCRC) = ((uint16_t)(wCRC) >> 8) ^
             wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
  }
  return wCRC;
}

uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2))
  {
    return false;
  }
  wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] &&
          ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2))
  {
    return;
  }
  wCRC = Get_CRC16_Check_Sum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
  pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

} // namespace ne_io
