/*
 * keyhandler.c
 *
 *  Created on: 2022. szept. 17.
 *      Author: drCsabesz
 */

#include "keyhandler.h"
#include "button.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
UsbHidVdjController_t VdjCtrlReport;

ButtonHandler_t ButtonPlayPause =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonCue =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonSearchLeft =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonSearchRight =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonHold =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonEject =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonTrackForward =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonTrackBackward =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonTimeMode =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonMasterTempo =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonJet =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonWah =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonZip =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonSearchFw =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonSearchBw =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

void KEYHANDLER_ScanButtons(void)
{
  // Read buttons on S1: HOLD, TRKB and PLAY
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET );
  if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonHold.ActiveState )
  {
    if( ButtonHold.IsPressed == false )
    {
      ButtonHold.IsPressed = true;
      if(VdjCtrlReport.buttons.leftHold == true)
      {
        VdjCtrlReport.buttons.leftHold = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftHold = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonHold.IsPressed == true )
    {
      ButtonHold.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD1_GPIO_Port, KD1_Pin) == ButtonTrackBackward.ActiveState )
  {
    if( ButtonTrackBackward.IsPressed == false )
    {
      ButtonTrackBackward.IsPressed = true;
      if(VdjCtrlReport.buttons.leftTrackSearchBw == true)
      {
        VdjCtrlReport.buttons.leftTrackSearchBw = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftTrackSearchBw = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonTrackBackward.IsPressed == true )
    {
      ButtonTrackBackward.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD2_GPIO_Port, KD2_Pin) == ButtonPlayPause.ActiveState )
  {
    if( ButtonPlayPause.IsPressed == false )
    {
      ButtonPlayPause.IsPressed = true;
      if(VdjCtrlReport.buttons.leftPlay == true)
      {
        VdjCtrlReport.buttons.leftPlay = false;
        VdjCtrlReport.buttons.leftPause = true;
      }
      else
      {
        VdjCtrlReport.buttons.leftPlay = true;
        VdjCtrlReport.buttons.leftPause = false;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonPlayPause.IsPressed == true )
    {
      ButtonPlayPause.IsPressed = false;
    }
  }
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET );
  // **********************************************************
  // Read buttons on S2: TIMEMODE, TRACKFORWARD and CUE
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_SET );
  if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonTimeMode.ActiveState )
  {
    if( ButtonTimeMode.IsPressed == false )
    {
      ButtonTimeMode.IsPressed = true;
      if(VdjCtrlReport.buttons.leftTimeMode == true)
      {
        VdjCtrlReport.buttons.leftTimeMode = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftTimeMode = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonTimeMode.IsPressed == true )
    {
      ButtonTimeMode.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD1_GPIO_Port, KD1_Pin) == ButtonTrackForward.ActiveState )
  {
    if( ButtonTrackForward.IsPressed == false )
    {
      ButtonTrackForward.IsPressed = true;
      if(VdjCtrlReport.buttons.leftTrackSearchFw == true)
      {
        VdjCtrlReport.buttons.leftTrackSearchFw = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftTrackSearchFw = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonTrackForward.IsPressed == true )
    {
      ButtonTrackForward.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD2_GPIO_Port, KD2_Pin) == ButtonCue.ActiveState )
  {
    if( ButtonCue.IsPressed == false )
    {
      ButtonCue.IsPressed = true;
      if(VdjCtrlReport.buttons.leftCue == true)
      {
        VdjCtrlReport.buttons.leftCue = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftCue = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonCue.IsPressed == true )
    {
      ButtonCue.IsPressed = false;
    }
  }
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET );
  // **********************************************************
  // Read buttons on S3: EJECT, JET és SEARCHBACKWARD
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET );
  if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonEject.ActiveState )
  {
    if( ButtonEject.IsPressed == false )
    {
      ButtonEject.IsPressed = true;
      if(VdjCtrlReport.buttons.leftEject == true)
      {
        VdjCtrlReport.buttons.leftEject = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftEject = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonEject.IsPressed == true )
    {
      ButtonEject.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD1_GPIO_Port, KD1_Pin) == ButtonJet.ActiveState )
  {
    if( ButtonJet.IsPressed == false )
    {
      ButtonJet.IsPressed = true;
      if(VdjCtrlReport.buttons.leftJet == true)
      {
        VdjCtrlReport.buttons.leftJet = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftJet = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonJet.IsPressed == true )
    {
      ButtonJet.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD2_GPIO_Port, KD2_Pin) == ButtonSearchBw.ActiveState )
  {
    if( ButtonSearchBw.IsPressed == false )
    {
      ButtonSearchBw.IsPressed = true;
      if(VdjCtrlReport.buttons.leftSearchBw == true)
      {
        VdjCtrlReport.buttons.leftSearchBw = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftSearchBw = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonSearchBw.IsPressed == true )
    {
      ButtonSearchBw.IsPressed = false;
    }
  }
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET );
  // **********************************************************
  // Read buttons on S4: MASTERTEMPO, ZIP and SEARCHFORWARD
  HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_SET );
  if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonMasterTempo.ActiveState )
  {
    if( ButtonMasterTempo.IsPressed == false )
    {
      ButtonMasterTempo.IsPressed = true;
      if(VdjCtrlReport.buttons.leftMasterTempo == true)
      {
        VdjCtrlReport.buttons.leftMasterTempo = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftMasterTempo = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonMasterTempo.IsPressed == true )
    {
      ButtonMasterTempo.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD1_GPIO_Port, KD1_Pin) == ButtonZip.ActiveState )
  {
    if( ButtonZip.IsPressed == false )
    {
      ButtonZip.IsPressed = true;
      if(VdjCtrlReport.buttons.leftZip == true)
      {
        VdjCtrlReport.buttons.leftZip = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftZip = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonZip.IsPressed == true )
    {
      ButtonZip.IsPressed = false;
    }
  }
  if( HAL_GPIO_ReadPin(KD2_GPIO_Port, KD2_Pin) == ButtonSearchFw.ActiveState )
  {
    if( ButtonSearchFw.IsPressed == false )
    {
      ButtonSearchFw.IsPressed = true;
      if(VdjCtrlReport.buttons.leftSearchFw == true)
      {
        VdjCtrlReport.buttons.leftSearchFw = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftSearchFw = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonSearchFw.IsPressed == true )
    {
      ButtonSearchFw.IsPressed = false;
    }
  }
  HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_RESET );
  // **********************************************************
  // Read buttons on S5: WAH
  HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, GPIO_PIN_SET );
  if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonWah.ActiveState )
  {
    if( ButtonWah.IsPressed == false )
    {
      ButtonWah.IsPressed = true;
      if(VdjCtrlReport.buttons.leftWah == true)
      {
        VdjCtrlReport.buttons.leftWah = false;
      }
      else
      {
        VdjCtrlReport.buttons.leftWah = true;
      }
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
  }
  else
  {
    if( ButtonWah.IsPressed == true )
    {
      ButtonWah.IsPressed = false;
    }
  }
  HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, GPIO_PIN_RESET );
}
