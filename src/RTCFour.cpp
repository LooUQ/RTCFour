/*
  RTC library for Arduino Zero.
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

/*
	RTC library for SAM D5x/E5x Family devices
	Copyright (c) 2019 LooUQ Incorporated

	\author Jensen Miller
	\date	June 2019
	\company LooUQ Incorporated
	\file	RTCFour.cpp

	Notes:
		The RTCZero library targeted the m0/m0+ family of devices, such as the Arduino
		Zero, Feather M0, etc. The newer chips separate the power management functionalities into
		more fitting categories.

	Changes:
		PM->APBA	::	MCLK->APBA		APBA is now connected to Main Clock
		PM->RCAUSE	::	RSTC->RCAUSE	RCAUSE is now in Reset Controller
		RTC->CTRL	::	RTC->CTRLA		Control Register is broken up 
					::	RTC->CTRLB			into two registers - A/B
		RTC->READREQ::	???				Read Required Register not available!

	\description The SAM D5x/E5x rearranges the clock modules and routes. The XOSC32K
		oscillator is directly routed to the RTC. Therefore, only the 1KHz source on
		oscillator needs to be enabled and sourced to the RTC. The RTC then divides
		the signal by 1024 via prescaler.

*/

#include <time.h>

#include "RTCFour.h"

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900

// Default date & time after reset
#define DEFAULT_YEAR    2000    // 2000..2063
#define DEFAULT_MONTH   1       // 1..12
#define DEFAULT_DAY     1       // 1..31
#define DEFAULT_HOUR    0       // 1..23
#define DEFAULT_MINUTE  0       // 0..59
#define DEFAULT_SECOND  0       // 0..59

voidFuncPtr RTC_callBack = NULL;

RTCFour::RTCFour()
{
  _configured = false;
}

void RTCFour::begin(bool resetTime)
{
  uint16_t tmp_reg = 0;
  
  // Configure the XOSC32K 1KHz output
  configure_1k_OSC();

  // If the RTC is in clock mode and the reset was
  // not due to POR or BOD, preserve the clock time
  // POR causes a reset anyway, BOD behaviour is?
  bool validTime = false;
  RTC_MODE2_CLOCK_Type oldTime;

  if ((!resetTime) && (RSTC->RCAUSE.reg & (RSTC_RCAUSE_SYST | RSTC_RCAUSE_WDT | RSTC_RCAUSE_EXT))) {
    if (RTC->MODE2.CTRLA.reg & RTC_MODE2_CTRLA_MODE_CLOCK) {
      validTime = true;
      oldTime.reg = RTC->MODE2.CLOCK.reg;
    }
  }

  // Select the 1k clock as source to the RTC
  select_rtc_clock();

  RTCdisable();

  RTCreset();

  tmp_reg |= RTC_MODE2_CTRLA_MODE_CLOCK; // set clock operating mode
  tmp_reg |= RTC_MODE2_CTRLA_PRESCALER_DIV1024; // set prescaler to 1024 for MODE2
  tmp_reg &= ~RTC_MODE2_CTRLA_MATCHCLR; // disable clear on match
  
  //According to the datasheet RTC_MODE2_CTRL_CLKREP = 0 for 24h
  tmp_reg &= ~RTC_MODE2_CTRLA_CLKREP; // 24h time representation

  /// !!! REMOVED !!!	RTC->MODE2.READREQ.reg &= ~RTC_READREQ_RCONT; // disable continuously mode

  RTC->MODE2.CTRLA.reg = tmp_reg;
  while (RTCisSyncing())
    ;

  NVIC_EnableIRQ(RTC_IRQn); // enable RTC interrupt 
  NVIC_SetPriority(RTC_IRQn, 0x00);

  RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0; // enable alarm interrupt
  RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = MATCH_OFF; // default alarm match is off (disabled)
  
  while (RTCisSyncing())
    ;

  RTCenable();
  RTCresetRemove();

  // If desired and valid, restore the time value, else use first valid time value
  if ((!resetTime) && (validTime) && (oldTime.reg != 0L)) {
    RTC->MODE2.CLOCK.reg = oldTime.reg;
  }
  else {
    RTC->MODE2.CLOCK.reg = RTC_MODE2_CLOCK_YEAR(DEFAULT_YEAR - 2000) | RTC_MODE2_CLOCK_MONTH(DEFAULT_MONTH) 
        | RTC_MODE2_CLOCK_DAY(DEFAULT_DAY) | RTC_MODE2_CLOCK_HOUR(DEFAULT_HOUR) 
        | RTC_MODE2_CLOCK_MINUTE(DEFAULT_MINUTE) | RTC_MODE2_CLOCK_SECOND(DEFAULT_SECOND);
  }
  while (RTCisSyncing())
    ;

  _configured = true;
}

void RTC_Handler(void)
{
  if (RTC_callBack != NULL) {
    RTC_callBack();
  }

  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // must clear flag at end
}

void RTCFour::enableAlarm(Alarm_Match match)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = match;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::disableAlarm()
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = 0x00;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::attachInterrupt(voidFuncPtr callback)
{
  RTC_callBack = callback;
}

void RTCFour::detachInterrupt()
{
  RTC_callBack = NULL;
}

void RTCFour::standbyMode()
{
  // Entering standby mode when connected
  // via the native USB port causes issues.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
}

/*
 * Get Functions
 */

uint8_t RTCFour::getSeconds()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.SECOND;
}

uint8_t RTCFour::getMinutes()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t RTCFour::getHours()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.HOUR;
}

uint8_t RTCFour::getDay()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.DAY;
}

uint8_t RTCFour::getMonth()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.MONTH;
}

uint8_t RTCFour::getYear()
{
  RTCreadRequest();
  return RTC->MODE2.CLOCK.bit.YEAR;
}

uint8_t RTCFour::getAlarmSeconds()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
}

uint8_t RTCFour::getAlarmMinutes()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

uint8_t RTCFour::getAlarmHours()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
}

uint8_t RTCFour::getAlarmDay()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
}

uint8_t RTCFour::getAlarmMonth()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
}

uint8_t RTCFour::getAlarmYear()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
}

/*
 * Set Functions
 */

void RTCFour::setSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.SECOND = seconds;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MINUTE = minutes;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.HOUR = hours;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    setSeconds(seconds);
    setMinutes(minutes);
    setHours(hours);
  }
}

void RTCFour::setDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.DAY = day;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MONTH = month;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.YEAR = year;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setDate(uint8_t day, uint8_t month, uint8_t year)
{
  if (_configured) {
    setDay(day);
    setMonth(month);
    setYear(year);
  }
}

void RTCFour::setAlarmSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    setAlarmSeconds(seconds);
    setAlarmMinutes(minutes);
    setAlarmHours(hours);
  }
}

void RTCFour::setAlarmDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
  if (_configured) {
    setAlarmDay(day);
    setAlarmMonth(month);
    setAlarmYear(year);
  }
}

uint32_t RTCFour::getEpoch()
{
  RTCreadRequest();
  RTC_MODE2_CLOCK_Type clockTime;
  clockTime.reg = RTC->MODE2.CLOCK.reg;

  struct tm tm;

  tm.tm_isdst = -1;
  tm.tm_yday = 0;
  tm.tm_wday = 0;
  tm.tm_year = clockTime.bit.YEAR + EPOCH_TIME_YEAR_OFF;
  tm.tm_mon = clockTime.bit.MONTH - 1;
  tm.tm_mday = clockTime.bit.DAY;
  tm.tm_hour = clockTime.bit.HOUR;
  tm.tm_min = clockTime.bit.MINUTE;
  tm.tm_sec = clockTime.bit.SECOND;

  return mktime(&tm);
}

uint32_t RTCFour::getY2kEpoch()
{
  return (getEpoch() - EPOCH_TIME_OFF);
}

void RTCFour::setAlarmEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    setAlarmDate(tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year - EPOCH_TIME_YEAR_OFF);
    setAlarmTime(tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
  }
}

void RTCFour::setEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    RTC->MODE2.CLOCK.bit.YEAR = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
    RTC->MODE2.CLOCK.bit.MONTH = tmp->tm_mon + 1;
    RTC->MODE2.CLOCK.bit.DAY = tmp->tm_mday;
    RTC->MODE2.CLOCK.bit.HOUR = tmp->tm_hour;
    RTC->MODE2.CLOCK.bit.MINUTE = tmp->tm_min;
    RTC->MODE2.CLOCK.bit.SECOND = tmp->tm_sec;

    while (RTCisSyncing())
      ;
  }
}

void RTCFour::setY2kEpoch(uint32_t ts)
{
  if (_configured) {
    setEpoch(ts + EPOCH_TIME_OFF);
  }
}

/**
 *	\brief Select RTC clock source.
 */
void RTCFour::select_rtc_clock() {
	/* Use XOSC1K from 32KHz external oscillator */
	OSC32KCTRL->RTCCTRL.bit.RTCSEL = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val;
}

/*
 * Private Utility Functions
 */


/**
 *	\brief Configure 1.024KHz Clock via OSC32K Controller
 */
void RTCFour::configure_1k_OSC()
{
	/* Behavior when a peripheral clock request is detected */
	OSC32KCTRL->XOSC32K.bit.ONDEMAND = 0b1;
	/* Run in standby during sleep mode */
	OSC32KCTRL->XOSC32K.bit.RUNSTDBY = 0b1;
	/* Enable 1KHz Output */
	OSC32KCTRL->XOSC32K.bit.EN1K = 0b1;
	/* Enable Crystal Oscillator */
	OSC32KCTRL->XOSC32K.bit.XTALEN = 0b1;
	/* Oscillator Startup time (~2000ms) */
	OSC32KCTRL->XOSC32K.bit.STARTUP = 0x4;
	/* Enable Oscillator */
	OSC32KCTRL->XOSC32K.bit.ENABLE = 0b1;
}

/* Synchronise the CLOCK register for reading*/
inline void RTCFour::RTCreadRequest() {
  if (_configured) {
    /// !!! REMOVED !!! RTC->MODE2.READREQ.reg = RTC_READREQ_RREQ;
    while (RTCisSyncing())
      ;
  }
}

/* Wait for sync in write operations */
inline bool RTCFour::RTCisSyncing()
{
  return (RTC->MODE2.SYNCBUSY.bit.CLOCK);
}

void RTCFour::RTCdisable()
{
	RTC->MODE2.CTRLA.bit.ENABLE = 0b0;
	while (RTCisSyncing())
	;
}

void RTCFour::RTCenable()
{
	RTC->MODE2.CTRLA.bit.ENABLE = 0b0; // enable RTC
	while (RTCisSyncing())
	;
}

void RTCFour::RTCreset()
{
	RTC->MODE2.CTRLA.bit.SWRST = 0b1; // software reset
	while (RTCisSyncing())
	;
}

void RTCFour::RTCresetRemove()
{
	RTC->MODE2.CTRLA.bit.SWRST = 0b0; // software reset remove
	while (RTCisSyncing())
	;
}
