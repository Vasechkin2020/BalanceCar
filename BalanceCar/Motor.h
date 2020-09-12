
#define AIN1 3
#define AIN2 11
#define PWMA 5

#define BIN1 4
#define BIN2 2
#define PWMB 6
#define STANBY 7

#define ENCODER_L 13
#define ENCODER_R 10

#define BUZZER_PIN 9
#define RGB_PIN A3

#define ECHO_PIN A1
#define TRIG_PIN A0

#define IR_PIN 8


//Для примера работы с битами
//#define spi_mosi_1   PORTB   |= 1<< 3    // Записываем в регистр PB3 единицу, это значит что на этом пине будет на выходе 1
//#define spi_mosi_0   PORTB   &= ~(1<< 3) // Записываем в регистр PB3 ноль, это значит что на этом пине будет на выходе 0
//#define spi_mosi_out DDRB    |= 1<< 3    // Записываем в регистр PB3 единицу, это значит что этот пин будет на выход

#define ain1_high PORTD   |= 1<< 3    // Записываем в регистр PD3 единицу, это значит что на этом пине будет на выходе 1
#define ain1_low  PORTD   &= ~(1<< 3)    // Записываем в регистр PD3 ноль, это значит что на этом пине будет на выходе 0

#define ain2_high PORTB   |= 1<< 3    // Записываем в регистр PB3 единицу, это значит что на этом пине будет на выходе 1
#define ain2_low  PORTB   &= ~(1<< 3)    // Записываем в регистр PB3 ноль, это значит что на этом пине будет на выходе 0

#define bin1_high PORTD   |= 1<< 4    // Записываем в регистр PD3 единицу, это значит что на этом пине будет на выходе 1
#define bin1_low  PORTD   &= ~(1<< 4)    // Записываем в регистр PD3 ноль, это значит что на этом пине будет на выходе 0

#define bin2_high PORTD   |= 1<< 2    // Записываем в регистр PD3 единицу, это значит что на этом пине будет на выходе 1
#define bin2_low  PORTD   &= ~(1<< 2)    // Записываем в регистр PD3 ноль, это значит что на этом пине будет на выходе 0



int SpeedShimL;
int SpeedShimR;
double MotorRightPulse, MotorLeftPulse;


float PWM;




struct my_struct
{
	float Angle_Complem;  // 4 байта
	float Angle_Kalman;  // 4 байта

	float speed_L;  // 4 byte	скорость движения линейная	в метрах в секунды
	float speed_R;  // 4 byte
	float Speed_raw;  //   Скорость с энкодера
	float Speed_raw_pwm;  //   Скорость с энкодера		  переведенная в ШИМ (pwm)
	float Speed_average;  // Скорость отфильтрованная
	float Speed_average_pwm;  // Скорость отфильтрованная переведенная в ШИМ (pwm)

	float way_l;	  // пройденный путь с момента запуска в метрах 
	float way_r;
	float Way_raw;  //  Путь с энкодера за 1 измерение
	float Way_average;  //  Путь с энкодера за 1 измерение по усредненной скорости



	int rpm_l;		 //Частота работы мотора
	int rpm_r;
	int pwm_L_raschet;   // 1 byte Скорость вращения колес переведеенная в ШИМ
	int pwm_R_raschet;   // 1 byte Скорость вращения колес переведеенная в ШИМ

};          // 30 байт

my_struct Robot;




void Init_Motor(void)
{
	//keep TB6612 AIN stop
	pinMode(AIN1, OUTPUT);
	digitalWrite(AIN1, LOW);
	pinMode(AIN2, OUTPUT);
	digitalWrite(AIN2, LOW);
	pinMode(PWMA, OUTPUT);
	digitalWrite(PWMA, HIGH);

	//keep TB6612 BIN stop
	pinMode(BIN1, OUTPUT);
	digitalWrite(BIN1, LOW);
	pinMode(BIN2, OUTPUT);
	digitalWrite(BIN2, LOW);
	pinMode(PWMB, OUTPUT);
	digitalWrite(PWMB, HIGH);

	//keep TB6612 Standby
	pinMode(STANBY, OUTPUT);
	digitalWrite(STANBY, HIGH);

	//encoder pin input
	pinMode(ENCODER_L, INPUT);
	digitalWrite(ENCODER_L, HIGH);  // включить подтягивающий резистор
	pinMode(ENCODER_R, INPUT);
	digitalWrite(ENCODER_R, HIGH);  // включить подтягивающий резистор

	delay(100);
}

//void Read_Encoder(void)
//{
//	int leftpluse = EncoderLeftPulse;
//	int rightpluse = EncoderRightPulse;
//
//	EncoderLeftPulse = 0;
//	EncoderRightPulse = 0;
//
//	// Определяем в какую сторону вращаются колеса
//	if (SpeedShimR < 0 && SpeedShimL < 0)
//	{
//		rightpluse = -rightpluse;
//		leftpluse = -leftpluse;
//	}
//	if (SpeedShimR > 0 && SpeedShimL > 0)
//	{
//		rightpluse = rightpluse;
//		leftpluse = leftpluse;
//	}
//	if (SpeedShimR < 0 && SpeedShimL > 0)
//	{
//		rightpluse = -rightpluse;
//		leftpluse = leftpluse;
//	}
//	if (SpeedShimR > 0 && SpeedShimL < 0)
//	{
//		rightpluse = rightpluse;
//		leftpluse = -leftpluse;
//	}
//	MotorRightPulse += rightpluse;
//	MotorLeftPulse += leftpluse;
//
//	static float speed_old_L = 0;
//	static float speed_old_R = 0;
//
//	float speed_L = speed_old_L * 0.25f + MotorLeftPulse * 0.75f;   // фильтруем значения и получаем число энкодеров за интервал измерения 
//	float speed_R = speed_old_R * 0.25f + MotorRightPulse * 0.75f;   // фильтруем значения и получаем число энкодеров за интервал измерения 
//
//	speed_old_L = speed_L;
//	speed_old_R = speed_R;
//
//	MotorRightPulse = MotorLeftPulse = 0;
//
//	Robot.speed_L = speed_L * 360 / 22 / 30;                //Переводим в угловую скорость градусы за интервал измерения 1 энкодер равен 360/22/30 = 0,545455 градуса 
//	Robot.speed_R = speed_R * 360 / 22 / 30;                //Переводим в угловую скорость градусы за интервал измерения 1 энкодер равен 360/22/30 = 0,545455 градуса 
//
//	Robot.position += (Robot.speed_L + Robot.speed_R) / 2.0f;   // Считаем позицию как на сколько градусов мы уехали при делении на 360 получис сколько оборотов и умножив на 2 пи R получим путь
//																// суммируем скорость и получаем пройденный путь и это позиция куда уехали
//
//	Robot.position = constrain(Robot.position, -3600, 3600);  // ограничиваем, значит вернемся только на 500 энкодеров
//}

float intervalEncoder = 0;	// Для расчета интревала между опросами энкодера	  в секундах
float now_time;			   //Переменная для запоминания текущего времени

void Read_Encoder(void)
{

	static float pre_intervalEncoder = micros();	// работает только 1 раз така как статик 
	now_time = micros();               // Считываем текушее время
	intervalEncoder = (now_time - pre_intervalEncoder)*0.000001;            // Находим разницу во времни с предыдцщим считыванием в секундах
	if (intervalEncoder < 0.001) intervalEncoder = 0.01;	  // Для первого запуска пока нет предыдущего значения
	pre_intervalEncoder = now_time;										 // Сохраняем время для следущего раза

	//Serial.print(" intervalEncoder0 ");Serial.println(intervalEncoder);
	Robot.rpm_l = EncoderLeftPulse * (1 / intervalEncoder)  * 60 / 22 / 30;	   // 11 окон энкодера умножаем на 2 так как и на подьем сигнала и на спад и редуктор 30 и 60 секунд так как интервал в секундах
	Robot.rpm_r = EncoderRightPulse * (1 / intervalEncoder) * 60 / 22 / 30;


	EncoderLeftPulse = EncoderRightPulse = 0;				//Обнуляем энкодеры
	//
	//Robot.pwm_L_raschet = map(Robot.rpm_l, 0, 330, 0, 255);     // Преобразование частоты вращения в ШИМ
	//Robot.pwm_R_raschet = map(Robot.rpm_r, 0, 330, 0, 255);     // Преобразование частоты вращения в ШИМ

	Robot.speed_L = (float)Robot.rpm_l / 60 * PI * 67 / 1000; // Частота вращения в секунды умножаем на Пи на диаметр колеса 67 мм и переводим в метры
	Robot.speed_R = (float)Robot.rpm_r / 60 * PI * 67 / 1000; // Частота вращения в секунды умножаем на Пи на диаметр колеса 67 мм и переводим в метры

	//Serial.print(Robot.speed_L); Serial.print(":");
	//Serial.print(Robot.speed_R); Serial.println("=");

	if (SpeedShimL < 0)
	{
		Robot.speed_L = - Robot.speed_L;		// Если шим с минусом то и скорость орицательная. едем назад
		//Robot.pwm_L_raschet = - Robot.pwm_L_raschet;
	}
	if (SpeedShimR < 0)
	{
		Robot.speed_R = - Robot.speed_R;		// Если шим с минусом то и скорость орицательная. едем назад
		//Robot.pwm_R_raschet = - Robot.pwm_R_raschet;
	}

	//Robot.way_l += (float)Robot.speed_L * intervalEncoder;		  // суммируем пройденный путь с момента запуска с учетом куда ехали вперед или назад
	//Robot.way_r += (float)Robot.speed_R * intervalEncoder;		  // суммируем пройденный путь с момента запуска с учетом куда ехали вперед или назад

	Robot.Speed_raw = (Robot.speed_L + Robot.speed_R)*0.5;		 // Скорость средняя по двум колесам
	Robot.Speed_raw_pwm = map(Robot.Speed_raw * 100, -113, 113, -25500, 25500) / 100;     // Преобразование отфильтрованной скорости  в ШИМ умножаем на 100 так как функция работает тольео с целыми числами
	Robot.Way_raw += Robot.Speed_raw * intervalEncoder; 		   // суммируем пройденный путь с момента запуска с учетом куда ехали вперед или назад

	Robot.Speed_average = (Robot.Speed_average * 0.3) + (Robot.Speed_raw * 0.7); //Фильтруем скорость от скачков и провалов
	Robot.Speed_average_pwm = map(Robot.Speed_average*100, -113, 113, -25500, 25500)/100;     // Преобразование отфильтрованной скорости  в ШИМ
	Robot.Way_average += Robot.Speed_average * intervalEncoder; 		   // суммируем пройденный путь с момента запуска с учетом куда ехали вперед или назад

	//Robot.position += (Robot.speed_L + Robot.speed_R) / 2.0f;   // Считаем позицию как на сколько градусов мы уехали при делении на 360 получис сколько оборотов и умножив на 2 пи R получим путь
	//															// суммируем скорость и получаем пройденный путь и это позиция куда уехали

	//Robot.position = constrain(Robot.position, -3600, 3600);  // ограничиваем, значит вернемся только на 500 энкодеров
}


void LeftMotor(int shim)       // Функция управления правым моторомif (shim == 0)
{
	if (shim > 255) shim = 255;
	if (shim < -255) shim = -255;

	if (shim == 0)
	{
		SpeedShimL = 0;
		analogWrite(PWMB, SpeedShimL);
	}
	if (shim > 0)
	{
		//digitalWrite(BIN1, HIGH);
		//digitalWrite(BIN2, LOW);
		bin1_high;
		bin2_low;
		SpeedShimL = shim;
		analogWrite(PWMB, SpeedShimL);
	}
	if (shim < 0)
	{
		//digitalWrite(BIN1, LOW);
		//digitalWrite(BIN2, HIGH);
		bin1_low;
		bin2_high;
		SpeedShimL = shim;
		analogWrite(PWMB, -SpeedShimL);
	}
	//Serial.print(" SpeedShimL : "); Serial.print(SpeedShimL);
}

void RightMotor(int shim)       // Функция управления правым моторомif (shim == 0)
{
	if (shim > 255) shim = 255;
	if (shim < -255) shim = -255;

	if (shim == 0)
	{
		SpeedShimR = 0;
		analogWrite(PWMA, SpeedShimR);
	}
	if (shim > 0)
	{
		//digitalWrite(AIN1, HIGH);
		//digitalWrite(AIN2, LOW);
		ain1_high;
		ain2_low;
		SpeedShimR = shim;
		analogWrite(PWMA, SpeedShimR);
	}
	if (shim < 0)
	{

		//digitalWrite(AIN1, LOW);
		//digitalWrite(AIN2, HIGH);
		ain1_low;
		ain2_high;
		SpeedShimR = shim;
		analogWrite(PWMA, -SpeedShimR);
	}
	//Serial.print(" SpeedShimR : "); Serial.print(SpeedShimR);
}

void Raspredelenie_PWM(int sum_pwm)
{
	float k = (float)Robot.rpm_l / Robot.rpm_r;
	//Serial.print(" kk= "); Serial.println(k,4);
	if (k > 1)
	{
		sum_pwmL = sum_pwm ;
		sum_pwmR = sum_pwm * k;
	}
	else
	{
		sum_pwmL = sum_pwmR = sum_pwm;
	}
}
void I2C_test()
{
	byte error, address;
	int nDevices;

	Serial.print("Scanning...");Serial.print(millis());

	nDevices = 0;
	for (address = 8; address < 127; address++) {
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print(" Address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.print(" !");

			nDevices++;
		}
		else if (error == 4) {
			Serial.print("Unknow error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C ааdevices found\n");
	else
	{
		Serial.print("done = ");
		Serial.println(nDevices);
	}

	//delay(1000);
}

void TestMotor()
{

	for (byte i = 0; i < 255; i++)
	{
		LeftMotor(i);		RightMotor(i);

		delay(50);
		Serial.print("PWM= "); Serial.print(i);
		Serial.print(" EncoderLeftPulse= "); Serial.print(EncoderLeftPulse);
		Serial.print(" EncoderRightPulse= "); Serial.println(EncoderRightPulse);
		float a = micros();
		Read_Encoder();
		//Serial.println(micros()-a);
		Serial.print(" Robot.rpm_l= "); Serial.print(Robot.rpm_l);
		Serial.print(" Robot.speed_L= "); Serial.print(Robot.speed_L);
		Serial.print(" Robot.way_l= "); Serial.print(Robot.way_l);
		Serial.print(" Robot.pwm_L_raschet = "); Serial.println(Robot.pwm_L_raschet);


		Serial.print(" Robot.rpm_r= "); Serial.print(Robot.rpm_r);
		Serial.print(" Robot.speed_R= "); Serial.print(Robot.speed_R);
		Serial.print(" Robot.way_r= "); Serial.print(Robot.way_r);
		Serial.print(" Robot.pwm_R_raschet = "); Serial.println(Robot.pwm_R_raschet);

	}

 }

//	float pid_P = 40, pid_D = 0.5, pid_Y = 0.75;
float target_angle = 0;
float delta_angle = 0;
float summa_angle = 0;
float pred_Angle = 0;     // Предыдущее значение угла отклонения

float PID_Angle(float angle_)
{
	static int pid_P = 50, pid_I = 0, pid_D = 100;
	float ret;

	delta_angle = angle_ - target_angle; //  Находим на сколько текщий угол отклоняеься от заданного
	summa_angle = summa_angle + delta_angle; // Интегральная часть

	// Устанавливаем смещение центра масс относительно центра. Отклонение вперед или назад
	//if (delta_angle > 1)
	//{
	//	target_angle += 0.0005;  // Если угол больше заданного более чем на 0,5 градуса то меняем заданный угол. Находим центр масс робота
	//}
	//if (delta_angle < -1)
	//{
	//	target_angle -= 0.0005;  // Если угол меньше заданного более чем на 0,5 градуса то меняем заданный угол. Находим центр масс робота
	//}

	// Устанавливаем пропорцианальный и диференциальный коэффициент и умножаем на угол отклонения и на скорость изменения угла в градусах
	ret = pid_P * delta_angle + (pid_I * summa_angle) + pid_D * (delta_angle - pred_Angle);
	pred_Angle = delta_angle;                    // Запоминаем значение как предыдущее для следующего раза расчета
	return ret;

}

long summa_way = 0;

float pred_Way = 0;     // Предыдущее значение пути

float PID_Way(float way_)
{															     
	static int pid_P = 600, pid_I = 0, pid_D = 5000;
	float ret;
	
	summa_way = summa_way + way_;			// Интегральная часть
	ret = pid_P * way_ + (pid_I * summa_way) + pid_D * (way_ - pred_Way);   //Второй пид регултор по скорости
	pred_Way = way_;
	return ret;
}
