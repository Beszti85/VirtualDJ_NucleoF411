/*
 * keyhandler.h
 *
 *  Created on: 2022. szept. 17.
 *      Author: drCsabesz
 */

#ifndef INC_KEYHANDLER_H_
#define INC_KEYHANDLER_H_

#include "main.h"

typedef struct
{
  uint8_t leftPlay           : 1;
  uint8_t leftPause          : 1;
  uint8_t leftCue            : 1;
  uint8_t leftStop           : 1;
  uint8_t leftHold           : 1;
  uint8_t leftJet            : 1;
  uint8_t leftZip            : 1;
  uint8_t leftWah            : 1;
  
  uint8_t leftTimeMode       : 1;
  uint8_t leftAutoCue        : 1;
  uint8_t leftMasterTempo    : 1;
  uint8_t leftEject          : 1;
  uint8_t leftTrackSearchFw  : 1;
  uint8_t leftTrackSearchBw  : 1;
  uint8_t leftSearchFw       : 1;
  uint8_t leftSearchBw       : 1;
  
  uint8_t rightPlay          : 1;
  uint8_t rightPause         : 1;
  uint8_t rightCue           : 1;
  uint8_t rightStop          : 1;
  uint8_t rightHold          : 1;
  uint8_t rightJet           : 1;
  uint8_t rightZip           : 1;
  uint8_t rightWah           : 1;
  
  uint8_t rightTimeMode      : 1;
  uint8_t rightAutoCue       : 1;
  uint8_t rightMasterTempo   : 1;
  uint8_t rightEject         : 1;
  uint8_t rightTrackSearchFw : 1;
  uint8_t rightTrackSearchBw : 1;
  uint8_t rightSearchFw      : 1;
  uint8_t rightSearchBw      : 1;
} Buttons_t;

typedef struct
{
  Buttons_t  buttons;
  uint8_t    leftPitch;
  uint8_t    rightPitch;
  uint8_t    leftVolume;
  uint8_t    rightVolume;
  uint8_t    crossFader;
  uint8_t    leftJog;
  uint8_t    rightJog;
} UsbHidVdjController_t;

void KEYHANDLER_ScanButtons(void);

#endif /* INC_KEYHANDLER_H_ */
