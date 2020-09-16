
#define Telemetria_define yes
//#define RemoteXY_define yes

#ifdef RemoteXY_define;
	#include "Remote_XY.h"
#endif 

long balance_pwm = 0;
float way_pwm = 0;
float speed_pwm = 0;
long position_pwm = 0;
int32_t sum_pwm = 0;
int sum_pwmL, sum_pwmR;  // Уточненный ШИМ на колесах с учетом что одно крутится быстрее другого даже при одинаковом ШИМ
float positions = 0;
float speed = 0;

#include "Wire.h"
#include "Kalman.h"
#include "Timer_ISR.h"
#include "Motor.h"
#include "MPU_6050.h"
#include "PrintLog.h"

void setup()
{
    Serial.begin(115200);
	Serial.println(" Start ");

#ifdef RemoteXY_define;
	RemoteXY_Init();
#endif 

	Init_Motor();

	Wire.begin();
	Wire.setClock(400000); // скорость передачи данных 400 кБит/с.
	
   // Разрешамм прерывания в общем в группах:
	PCICR = 0b00000001;   //Бит 0 в регистре PCICR отвечает за группу 0, бит 1 за группу 1, бит 2 за группу 2. Разрешили все три группы.
  // Теперь разрешим прерывание на в каждй группе, сделав запись в регистры PCMSK0...PCMSK2:
  // Тут разрешаем по 10 порту PB2 и 13 порту PB5
	PCMSK0 = 0b00110100;

	Timer1_Init();     // Таймер 1 на  милисекунд по совпадению А

	Serial.println(" initialize 6050 start ");
	I2C_test();
	Init_6050();
	Read_Colibrovka();
	//Colibrovka_6050();

	//TestMotor();

	//LeftMotor(150);  RightMotor(150);

	//delay(999999999999);
}


void loop() 
{
#ifdef RemoteXY_define;

		RemoteXY_Handler();				//обработчик связи
		//PrintRemoteXY_variable();	   //выводим на печать переменные которыми управляем 

#endif 

		if (flag_timer_encoder)			// Флаг по таймеру для обсчета энкодера и вычисления скорости движения, вращения колес и пройденного пути
		{
			flag_timer_encoder = false;
			Read_Encoder();
			//way_pwm = PID_Way(Robot.Way_average);	// Расчет ШИМ для компенсации по пути 
			speed_pwm = PID_Speed(Robot.Speed_raw);	// Расчет ШИМ для компенсации по скорости

		}

		if (flag_mpu6050 )
		{
			flag_mpu6050 = false;
			Read_6050();

			balance_pwm = PID_Angle(Robot.Angle_Kalman); // Расчет ШИМ по углу отклонения	

			sum_pwm = balance_pwm;        //только блансировка	
			sum_pwm = sum_pwm + speed_pwm;   // дополнительно второй пид регулятор по скорости, чтобы стоял на месте
			//sum_pwm = sum_pwm + Robot.Speed_average_pwm ;   // дополнительно учет (компенсация) скорости
			//sum_pwm = sum_pwm + way_pwm;   // дополнительно второй пид регулятор по пути для возврата в точку балансировки

			if (Robot.Angle_Complem > 30 || Robot.Angle_Complem < -30) sum_pwm = 0;         // Отключаем моторчики при большом наклоне

#ifdef RemoteXY_define;
			LeftMotor(sum_pwm + RemoteXY.joystick_1_x / 3);
			RightMotor(sum_pwm - RemoteXY.joystick_1_x / 3);
#else
			LeftMotor(sum_pwm);
			RightMotor(sum_pwm);
#endif 
		}

#ifdef Telemetria_define;

		if (flag_timer_telemetria)			// Флаг по таймеру для обсчета энкодера и вычисления скорости движения, вращения колес и пройденного пути
		{
			flag_timer_telemetria = false;
			PrintTelemetria();
		}
#endif 
}


