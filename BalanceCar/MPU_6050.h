#define MPU6050_Address 0x68

typedef struct
{
	float x;
	float y;
	float z;

}ST_MPU6050;

typedef struct
{
	int16_t ax,ay,az,gx,gy,gz;	   // raw данные
	float ax_data, ay_data, az_data, gx_data, gy_data, gz_data;	   //данные после поправок колибровки и делителей
	float ax_angle, ay_angle, az_angle, gx_angle, gy_angle, gz_angle;	   // вычисленные углы;

}ST_MPU6050_NEW;


ST_MPU6050_NEW MPU6050;
ST_MPU6050 Accel_Offset, Gyro_Offset;  // mpu6050 static


uint8_t buffer_acc_gyro[14];			 // Буфер для считывания данных аксельрометра

int16_t temp9255;
uint16_t acc_delitel, gyro_delitel;
volatile float intervalAccGyro, pred_time_interrupt_MPU6050;
float X_angle_comp;   // Значение комплементарного фильтра

Kalman AngleKalman;  // Перемеенная класса только что бы вызвать метод класса

void Init_6050()
{
	uint8_t WIA_MPU = ReadByte_I2C(MPU6050_Address, 0x75);     // Считываем значение регистра "Кто я"
	Serial.print("WIA_MPU: "); Serial.println(WIA_MPU, HEX);

	if (WIA_MPU == 0x71 | WIA_MPU == 0x73 | WIA_MPU == 0x68)          //71- 9050       73-9255
	{
		if (WIA_MPU == 0x68)  Serial.println(" Successfully connected to MPU6050");
		if (WIA_MPU == 0x71)  Serial.println(" Successfully connected to MPU9250");
		if (WIA_MPU == 0x73)  Serial.println(" Successfully connected to MPU9255");

		//	SPI.setClockDivider(SPI_CLOCK_DIV16);  // чтение SPI_CLOCK_DIV2  Устанавливаем скорость работы протокола SPI быстрее не поддерживает 9255

		WriteByte_I2C(MPU6050_Address, 0x6B, 0b10000000);                         // 107 регистр Reset MPU9255 
		delay(10);  // После  ждем 1 миллисекунду

		//WriteByte_SPI(MPU9255_CS, 0x6A, 0b00010000);                         // 106 регистр Отключение работы по I2C и отставление только SPI
		//delay(1);  // После  ждем 1 миллисекунду

		WriteByte_I2C(MPU6050_Address, 0x6B, 0b00000001);                         // 107 регистр Clock Source  Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
		delay(1);  // После  ждем 1 миллисекунду

		WriteByte_I2C(MPU6050_Address, 0x6C, 0b00000000);                         // 108 регистр Включаем гироскоп и акселерометр
		delay(1);  // После  ждем 1 миллисекунду
																		// 25 регистр SMPLRT_DIV Устанавливаем делитель внутренней частоты 
	//	WriteByte_I2C(MPU6050_Address, 0x19, 0b00000011);                         // устанавливаем HEX формате 3 в BIN (11) и получаем частоту 1 раз в 4 милисекунды или 250 Герц 
		WriteByte_I2C(MPU6050_Address, 0x19, 4);                         // устанавливаем HEX формате 7 в BIN (111) и получаем частоту 1 раз в 8 милисекунды или 125 Герц 

		delay(1);  // После  ждем 1 миллисекунду 

		//WriteByte_SPI(MPU9255_CS, 0x1C, 0b00000000);   acc_delitel = 16384;   //  +-2G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр
		WriteByte_I2C(MPU6050_Address, 0x1C, 0b00001000);   acc_delitel = 8192;    //  +-4G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр    
		//WriteByte_SPI(MPU9255_CS,0x1C,0b00010000);   acc_delitel = 4096;    //  +-8G      // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр   
		//WriteByte_SPI(MPU9255_CS,0x1C,0b00011000);     acc_delitel = 2048;  //  +-16G     // 28 регистр ACCEL_CONFIG Устанавливаем чувствительность Аксельрометр

		delay(1);  // После  ждем 1 миллисекунду

	//  WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001000);    //   460  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001001);    //   184  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001010);    //   92  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      
		WriteByte_I2C(MPU6050_Address, 0x1D, 0b00001011);    //   41  Bandwidth(Hz);     // 29 регистр ACCEL_CONFIG2 Устанавливаем ширину пропускания Аксельрометр      

		delay(1);  // После  ждем 1 миллисекунду

		//WriteByte_I2C(MPU6050_Address, 0x1B, 0b00000000);   gyro_delitel = 131;    //  250dps     // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
	  //WriteByte_SPI(MPU9255_CS, 0x1B, 0b00001000);   gyro_delitel = 65.5;   //  500dps     // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
		WriteByte_I2C(MPU6050_Address, 0x1B, 0b00010000);   gyro_delitel = 32.8;   //  1000dps    // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа
	 // WriteByte_SPI(MPU9255_CS, 0x1B, 0b00011000);   gyro_delitel = 16.4;     //  2000dps      // 27 регистр GYRO_CONFIG Устанавливаем чувствительность Гироскопа      
		delay(1);

		//WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000000);    //   250  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа      
	  //	WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000001);    //   184  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
	  //	WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000010);    //   92  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		WriteByte_I2C(MPU6050_Address, 0x1A, 0b00000011);    //   41  Bandwidth(Hz);     // 26 регистр CONFIG Устанавливаем ширину пропускания Гироскопа
		delay(1);

		WriteByte_I2C(MPU6050_Address, 0x37, 0b00010000);         // 55 регистр INT_PIN_CFG Устанавливаем очищение статура прерывания при любом чтении из регистра данных
//		WriteByte_SPI(MPU9255_CS, 0x37, 0b00000000);         // 55 регистр INT_PIN_CFG Устанавливаем очищение статура прерывания при любом чтении из регистра данных

		delay(1);

		WriteByte_I2C(MPU6050_Address, 0x38, 0b00000001);         // 56 регистр INT_PIN_CFG Включаем прерывание на пине
		delay(1);

		// Set by pass mode for the magnetometers
		//WriteByte_I2C(MPU6050_Address, 0x37, 0x02);		 //Включаем доступ к магнетрометру по I2C по этим же контактам. Типа соединяем внутреннюю шину и внешнюю .

	}
	else
	{
		Serial.println("Failed to Connect to MPU6050 !!!!!!!!!!!!!!");
		delay(100000);
	}


	//mpu.initialize();
}

float ComplementarAngleX(double AngelAccel_x, double GyroData_y)       //Простейший комплементарный фильтр
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.03;
	X_angle_comp = ((1 - coefficient) * (X_angle_comp + GyroData_y)) + (coefficient * AngelAccel_x);

	//Serial.print(" AngelAccel_x ");	Serial.print(AngelAccel_x);
	//Serial.print(" AngelGyro_y ");	Serial.print(AngelGyro_y);
	//Serial.print(" X_angle_comp ");	Serial.println(X_angle_comp);
	return X_angle_comp;
}


void Read_6050()  // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	Wire.beginTransmission(MPU6050_Address);
	int8_t registr = 0x3B;			  // Начало регистров последовательно 14 штук	начиная с этого
	Wire.write(registr);
	byte reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! 6050 Write Mistake reza = ");Serial.println(reza);
		//Serial2.print("!!! 9255 Write Mistake reza = ");Serial2.println(reza);
	};

	uint8_t buffer_acc_gyroTemp[14];			 // Буфер для считывания данных аксельрометра  временный
	byte count_byte = 14;			  // число запрашиваемых байт

	int rezb = Wire.requestFrom(MPU6050_Address, count_byte);
	//Serial2.print(rezb);
	if (rezb == count_byte)         // Если вернулось  столько сколько просили
	{
		for (byte i = 0;i < count_byte; i++)   // Считываем 14 байт uint16_t - это 7 двухбайтный значений uint16_t если такие данные есть
		{
			buffer_acc_gyro[i] = Wire.read();
		}
	}
	else
	{
		Serial.print("!!! 6050 Write Mistake rezb = ");Serial.println(rezb);
		//Serial2.print("!!! 9255 Write Mistake rezb = ");Serial2.println(rezb);
	}
	MPU6050.ax = (((int16_t)buffer_acc_gyro[0]) << 8) | buffer_acc_gyro[1];
	MPU6050.ay = (((int16_t)buffer_acc_gyro[2]) << 8) | buffer_acc_gyro[3];
	MPU6050.az = (((int16_t)buffer_acc_gyro[4]) << 8) | buffer_acc_gyro[5];
	temp9255   = (((int16_t)buffer_acc_gyro[6]) << 8) | buffer_acc_gyro[7];
	MPU6050.gx = (((int16_t)buffer_acc_gyro[8]) << 8) | buffer_acc_gyro[9];
	MPU6050.gy = (((int16_t)buffer_acc_gyro[10]) << 8) | buffer_acc_gyro[11];
	MPU6050.gz = (((int16_t)buffer_acc_gyro[12]) << 8) | buffer_acc_gyro[13];

	MPU6050.ax_data = (MPU6050.ax - Accel_Offset.x);// / acc_delitel;		  //Делить на делитель не обязательно, так как считаем отношение
	//MPU6050.ay_data = (MPU6050.ay - Accel_Offset.y) / mpu.AcceRatio;
	MPU6050.az_data = (MPU6050.az - Accel_Offset.z);// / mpu.AcceRatio;

	//Вычисляем углы
	MPU6050.ax_angle = atan2(MPU6050.ax_data, MPU6050.az_data) * RAD_TO_DEG;
	//MPU6050.ay_angle = atan2(MPU6050.ay_data, MPU6050.az_data) * RAD_TO_DEG;

	//Учитываем калибровку 
	//MPU6050.gx_data = (MPU6050.gx - Gyro_Offset.x) / gyro_delitel;
	MPU6050.gy_data = -(MPU6050.gy - Gyro_Offset.y) / gyro_delitel;
	//MPU6050.gz_data = (MPU6050.gz - Gyro_Offset.z) / gyro_delitel;

	intervalAccGyro = (time_interrupt_MPU6050 - pred_time_interrupt_MPU6050)*0.000001;	 	// Измеряем время прошедшее с предудущего формирования данных гироскопа в секундах
	pred_time_interrupt_MPU6050 = time_interrupt_MPU6050;

	//Скорость по гироскопу умножаем на прошедшее время и получаем угол на который за это время отклонился
	//MPU6050.ax_angle += MPU6050.gx_data * intervalAccGyro;
	MPU6050.ay_angle += MPU6050.gy_data * intervalAccGyro;
	//MPU6050.az_angle += MPU6050.gz_data * intervalAccGyro;

	Robot.Angle_Complem = ComplementarAngleX(MPU6050.ax_angle, MPU6050.gy_data * intervalAccGyro);
	Robot.Angle_Kalman = AngleKalman.getAngle(MPU6050.ax_angle, MPU6050.gy_data, intervalAccGyro);
	//Serial.print(" io= "); Serial.println(intervalAccGyro,6);


}


void Read_Colibrovka()	   //Присваиваем значения по заранее отколиброванным
{
	Accel_Offset.x = 386;
	Accel_Offset.y = -148;
	Accel_Offset.z = 291;

	Gyro_Offset.x = -29;
	Gyro_Offset.y = 17;
	Gyro_Offset.z = -79;
	delay(20);
	Read_6050();
	X_angle_comp = MPU6050.ax_angle; //Начальное значение комплементарного фильтра как у аксельрометра
	AngleKalman.setAngle(MPU6050.ax_angle); //Начальное значение фильтра Калмана как у аксельрометра
}

void Colibrovka_6050()
{
	Accel_Offset.x = 0;
	Accel_Offset.y = 0;
	Accel_Offset.z = 0;
	Gyro_Offset.x = 0;
	Gyro_Offset.y = 0;
	Gyro_Offset.z = 0;

	int32_t ax_zero = 0, ay_zero = 0, az_zero = 0, gx_zero = 0, gy_zero = 0, gz_zero = 0;
	int count = 1000;

	for (int i = 0; i < count; i++) {
		Read_6050();
		ax_zero += MPU6050.ax;
		ay_zero += MPU6050.ay;
		az_zero += (MPU6050.az - acc_delitel);


		gx_zero += MPU6050.gx;
		gy_zero += MPU6050.gy;
		gz_zero += MPU6050.gz;
		delay(10);
	}
	Accel_Offset.x = (float)ax_zero / count;
	Accel_Offset.y = (float)ay_zero / count;
	Accel_Offset.z = (float)az_zero / count;
	Gyro_Offset.x = (float)gx_zero / count;
	Gyro_Offset.y = (float)gy_zero / count;
	Gyro_Offset.z = (float)gz_zero / count;

	Serial.println("Accel_Offset :");
	Serial.println(Accel_Offset.x);
	Serial.println(Accel_Offset.y);
	Serial.println(Accel_Offset.z);
	Serial.println("Gyro_Offset :");
	Serial.println(Gyro_Offset.x);
	Serial.println(Gyro_Offset.y);
	Serial.println(Gyro_Offset.z);


	//Accel_Offset.x = 1306;
	//Accel_Offset.y = 205;
	//Accel_Offset.z = -708;

	//Gyro_Offset.x = - 185;
	//Gyro_Offset.y = + 15;
	//Gyro_Offset.z =  -296;



	delay(500000);
}

