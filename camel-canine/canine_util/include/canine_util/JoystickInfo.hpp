//
// Created by camel on 22. 9. 21.
//

#ifndef QTTCPCLIENT_JOYSTICKINFO_H
#define QTTCPCLIENT_JOYSTICKINFO_H

typedef struct _AXIS_
{
    float LeftStickX;
    float LeftStickY;
    float LeftTrigger;
    float RightStickX;
    float RightStickY;
    float RightTrigger;
    float DpadX;
    float DpadY;
} AXIS, * pAXIS;

typedef struct _BUTTON_
{
    float FaceButtonA;
    float FaceButtonB;
    float FaceButtonX;
    float FaceButtonY;
    float LeftBumper;
    float RightBumper;
    float Back;
    float Start;
    float Guide;
    float LeftStick;
    float RightStick;
} BUTTON, * pBUTTON;


#endif //QTTCPCLIENT_JOYSTICKINFO_H
