/*
   This source code of graphical user interface
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/
	 - for ANDROID 4.5.1 or later version;
	 - for iOS 1.4.1 or later version;

   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
{ 255,4,0,0,0,39,0,10,13,0,
1,0,12,33,18,18,2,31,88,0,
2,0,10,7,22,11,2,26,31,31,
79,78,0,79,70,70,0,5,32,44,
7,50,50,2,26,31 };

// структура определяет все переменные и события вашего интерфейса управления 
struct {

	// input variables
	uint8_t button_1; // =1 если кнопка нажата, иначе =0 
	uint8_t switch_1; // =1 если переключатель включен и =0 если отключен 
	int8_t joystick_1_x; // =-100..100 координата x положения джойстика 
	int8_t joystick_1_y; // =-100..100 координата y положения джойстика 

	  // other variable
	uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

void PrintRemoteXY_variable()
{
	Serial.print(" button_1 = ");Serial.print(RemoteXY.button_1);
	Serial.print(" switch_1 = ");Serial.print(RemoteXY.switch_1);

	Serial.print(" joystick_1_x = ");Serial.print(RemoteXY.joystick_1_x);
	Serial.print(" joystick_1_y = ");Serial.print(RemoteXY.joystick_1_y);

	Serial.print(" Connect = ");Serial.println(RemoteXY.connect_flag);
}
