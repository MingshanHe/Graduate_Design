///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2013 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo for USB-to-CAN V2

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file EciDemo113.h
*/

#ifndef __ECIDEMO113_H__
#define __ECIDEMO113_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include <device_driver/eci/ECI113.h>

#include <device_driver/eci/ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

//////////////////////////////////////////////////////////////////////////
// data types

#include <device_driver/eci/ECI_poppack.h>

//////////////////////////////////////////////////////////////////////////
// static function prototypes
ECI_RESULT Can_Tx_Data(ECI_RESULT hResult,BYTE tx_data[6][8],DWORD *Move_motorID);

ECI_RESULT Can_Rx_Position(ECI_RESULT hResult,BYTE tx_data[6][8],DWORD *Move_motorID);

ECI_RESULT Can_Rx_Velocity(ECI_RESULT hResult,BYTE tx_data[6][8],DWORD *Move_motorID);

ECI_RESULT Can_Rx_Torque(ECI_RESULT hResult,BYTE tx_data[6][8],DWORD *Move_motorID);

ECI_RESULT EciDemo113(void);

ECI_RESULT close_can_rtx_data(ECI_RESULT hResult);

#endif //__ECIDEMO113_H__
