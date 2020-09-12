static volatile long time_interrupt_MPU6050;
bool flag_mpu6050 = false;

volatile byte st_b2 = 0;
volatile byte st_b4 = 0;
volatile byte st_b5 = 0;

volatile byte tek_st_PB2 = 0, tek_st_PB5 = 0;		 // состояние тукущего статуса пина для энкодера

static volatile int EncoderRightPulse = 0, EncoderLeftPulse = 0;


static void Timer1_Init()     // Таймер 1 на 10 милисекунд по совпадению А
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B |= (1 << WGM12);                    // Режим CTC (сброс по совпадению) 
   // TCCR5B |= (1<<CS50);                       // Тактирование от CLK. 
															  // Если нужен предделитель : 
	// TCCR5B |= (1<<CS51);                   // CLK/8 
	TCCR1B |= (1 << CS10) | (1 << CS11); // CLK/64 //          Через 4 микросекунды считает 1/(16 000 000/64)
   // TCCR5B |= (1<<CS52);                   // CLK/256 
   // TCCR5B |= (1<<CS50)|(1<<CS52); // CLK/1024 
															// Верхняя граница счета. Диапазон от 0 до 65535. 
	OCR1A = 2499;    // Умножить на 4 микросекунды                         // Частота прерываний A будет = Fclk/(N*(1+OCR5A))  На 1 меньше так как считает от 0
  //  OCR5B = 15624;                                // Частота прерываний B будет = Fclk/(N*(1+OCR5B)) 
															 // где N - коэф. предделителя (1, 8, 64, 256 или 1024) 
	TIMSK1 = (1 << OCIE1A);                   // Разрешить прерывание по совпадению A 
   // TIMSK5 |= (1<<OCIE5B);                   // Разрешить прерывание по совпадению B 
  //  TIMSK5 |= (1<<TOIE5);                     // Разрешить прерывание по переполнению 
}


bool flag_timer_encoder = false;
bool flag_timer_telemetria = false;

byte count_timer_encoder = 0;
byte count_timer_telemetria = 0;

ISR(TIMER1_COMPA_vect)         // Обработчик прерывания таймера 4 по совпадению A 	1 раз в 10 милисекунд
{
	count_timer_encoder++;
	if (count_timer_encoder >= 5) // 50 milliseconds
	{
		count_timer_encoder = 0;
		flag_timer_encoder = true;
	}

	count_timer_telemetria++;
	if (count_timer_telemetria >= 5) // 50 milliseconds
	{
		count_timer_telemetria = 0;
		flag_timer_telemetria = true;
	}
}

ISR(PCINT0_vect)		   //Поскольку есть только одна фаза с энкодера то направление вращения определить только по энкодеру нельзя
{
	//long a = micros();
	//Прерывания на 10 и 13 пинах для энкодера
	tek_st_PB2 = PINB & (1 << PB2);		// Отпределяем значение пина 
	if (tek_st_PB2 != st_b2)			//Если статус на пине изменился
	{
		st_b2 = tek_st_PB2; //Запоминаем новый статус
		EncoderRightPulse++;   //Считаем импульсы
	}

	tek_st_PB5 = PINB & (1 << PB5);		// Отпределяем значение пина 
	if (tek_st_PB5 != st_b5)			//Если статус на пине изменился
	{
		st_b5 = tek_st_PB5; //Запоминаем новый статус
		EncoderLeftPulse++;   //Считаем импульсы
	}

	//   Прерывание на 12 пине для MPU6050

	if (!(PINB & (1 << PB4)) && st_b4 == 1)   // Если бит стал равен 0 и в предыдущем вызове был 1
	{
		st_b4 = 0;
	}
	if ((PINB & (1 << PB4)) && st_b4 == 0)   // Если бит стал равен 1 и в предыдущем вызове был 0
	{
		st_b4 = 1;
		flag_mpu6050 = true;
		time_interrupt_MPU6050 = micros();			//Запоминаем время в которое сработало прерывание для дальнейшего расчета гироскопа
		//Serial.print("Up ");Serial.print(millis());		
	}
	//Serial.println(micros()-a);
}


//ISR(PCINT0_vect)		 //Первый вариант работы логики энкодера. Вроде верный
//{
//	//long a = micros();
//	//Прерывания на 10 и 13 пинах для энкодера
//	if (!(PINB & (1 << PB2)) && st_b2 == 1)   // Если бит стал равен 0 и в предыдущем вызове был 1
//	{
//		st_b2 = 0;
//		EncoderRightPulse++;
//	}
//	if ((PINB & (1 << PB2)) && st_b2 == 0)   // Если бит стал равен 1 и в предыдущем вызове был 0
//	{
//		st_b2 = 1;
//		EncoderRightPulse++;
//	}
//	if (!(PINB & (1 << PB5)) && st_b5 == 1)   // Если бит стал равен 0 и в предыдущем вызове был 1
//	{
//		st_b5 = 0;
//		EncoderLeftPulse++;
//	}
//	if ((PINB & (1 << PB5)) && st_b5 == 0)   // Если бит стал равен 1 и в предыдущем вызове был 0
//	{
//		st_b5 = 1;
//		EncoderLeftPulse++;
//	}
//	//   Прерывание на 12 пине для MPU6050
//
//	if (!(PINB & (1 << PB4)) && st_b4 == 1)   // Если бит стал равен 0 и в предыдущем вызове был 1
//	{
//		st_b4 = 0;
//	}
//	if ((PINB & (1 << PB4)) && st_b4 == 0)   // Если бит стал равен 1 и в предыдущем вызове был 0
//	{
//		st_b4 = 1;
//		flag_mpu6050 = 1;
//		time_interrupt_MPU6050 = micros();			//Запоминаем время в которое сработало прерывание для дальнейшего расчета гироскопа
//		//Serial.print("Up ");Serial.print(millis());		
//	}
//	//Serial.println(micros()-a);
//}


uint8_t  ReadByte_I2C(uint8_t address, int8_t registr)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	byte reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake reza = ");Serial.println(reza);
		//Serial2.print("!!! ReadByte_I2C_WriteMistake reza = ");Serial2.println(reza);

	};
	byte rezb = Wire.requestFrom(address, (uint8_t)1);
	if (rezb == 1)
	{
		uint8_t data = Wire.read();             //read one byte of data
		return data;
	}
	else
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake rezb = ");Serial.println(rezb);
		//Serial2.print("!!! ReadByte_I2C_WriteMistake rezb = ");Serial2.println(rezb);

		return 0;
	}
}
void  WriteByte_I2C(uint8_t address, int8_t registr, uint8_t data)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(data);
	Wire.endTransmission();
}
