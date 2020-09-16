void PrintTelemetria()
{
	long a = millis();
	//Serial.print(Robot.Angle_Complem); Serial.print(";");
	Serial.print(Robot.Angle_Kalman); Serial.print(";  ");
	Serial.print(balance_pwm); Serial.print(";     ");

	//Serial.print(Robot.Speed_average); Serial.print(";");
	//Serial.print(Robot.Speed_average_pwm); Serial.print(";");
	Serial.print(Robot.Way_average); Serial.print("<=; ");
	//Serial.print(Robot.Speed_average_pwm); Serial.print(";");
	Serial.print(way_pwm,1); Serial.print(";    ");
	Serial.print(sum_pwm); Serial.print("; ");

	//Serial.print(target_angle); Serial.print(";");
	//Serial.print(delta_angle); Serial.print(";");
	//Serial.print(way_pwm); Serial.print(";");			 
	//Serial.print(sum_way_pwm); Serial.print(";");
	//Serial.print(PID_Way); Serial.print(";");


	//Serial.print(Robot.pwm_L_raschet); Serial.print(";");
	//Serial.print(Robot.pwm_R_raschet); Serial.print(";");
	long b = millis();

	Serial.println("");

}


void PrintRawData()
{
	Serial.println(" ");
	Serial.print("RAW -> ");
	//Serial.print(Accel.x);   Serial.print(" , ");
	////Serial.print(Accel.y);   Serial.print(" , ");
	////Serial.print(Accel.z);   Serial.print(" ;  !!  ;  ");
	////Serial.print(Gyro.x);   Serial.print(" , ");
	//Serial.print(Gyro.y);   Serial.print(" , ");
	//Serial.print(Gyro.z);   Serial.print(" ,  !!  ;  ");
	Serial.println(" ");

}
void PrintData()
{
	Serial.print(" Data -> ");
	//Serial.print(AccelData.x, 4);   Serial.print(" , ");
	////Serial.print(AccelData.y, 4);   Serial.print(" , ");
	////Serial.print(AccelData.z, 4);   Serial.print(" ;  !!  ");
	//Serial.print(GyroData.x, 4);   Serial.print(" , ");
	//Serial.print(GyroData.y, 4);   Serial.print(" , ");

	//Serial.print(GyroData.z, 4);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintAngle()
{
	Serial.print("Angle -> ");
	//Serial.print(AngelAccel.x);   Serial.print(" , ");
	//Serial.print(AngelGyro.y);   Serial.print(" , ");
	//Serial.print(AngelGyro.x);   Serial.print(" , ");

	Serial.print(Robot.Angle_Complem, 2);   Serial.print("  ");
	//Serial.print(mKalFilter.X_angle_kalm);   Serial.print(" ");
	//Serial.print(Robot.dAngle, 0);   Serial.print("  ");
	//Serial.print(GyroData.y, 0);   Serial.print(" ! ");

	//Serial.print(Robot.iAngle, 0);   Serial.print("  ");


	//Serial.print((Robot.speed_L + Robot.speed_R) / 2.0f);   Serial.print("  ");
	//Serial.print(balance_pwm);   Serial.print("  "); 
	//Serial.print(way_pwm);   Serial.print(" ! ");
	//Serial.print(sum_pwm);   Serial.print(" = ");

//	Serial.print(positions);   Serial.print("  ");
	//Serial.print(position_pwm);   Serial.print(" ! ");



//	Serial.print(EncoderLeftPulse);   Serial.print(" ");
	//Serial.print(Robot.speed_L);   Serial.print(" ");

//	Serial.print(EncoderRightPulse);   Serial.print(" ");
	//Serial.print(Robot.speed_R);   Serial.print(" ");

//	Serial.print(Robot.position);   Serial.print(" ");
	//Serial.print(AngelAccel.y);   Serial.print(" , ");
	//Serial.print(AngelGyro.x);   Serial.print(" , ");

	Serial.println(" ");
}
void PrintAngleGraf()
{
	//Serial.print(AngelAccel.x);   Serial.print(" , ");
	//Serial.print(AngelGyro.y);   Serial.print(" , ");
	//Serial.print(X_angle_comp);   Serial.print(" , ");

	//Serial.print(AngelAccel.y);   Serial.print(" , ");

	//Serial.print(AngelGyro.x);   Serial.print(" , ");
	Serial.println(" ");
}
void Print_motor()
{
	Serial.print(" EncoderLeftPulse= "); Serial.print(EncoderLeftPulse);
	Serial.print(" EncoderRightPulse= "); Serial.println(EncoderRightPulse);

	Serial.print(" SpeedShimL= "); Serial.print(SpeedShimL);
	Serial.print(" Robot.rpm_l= "); Serial.print(Robot.rpm_l);
	Serial.print(" Robot.speed_L= "); Serial.print(Robot.speed_L);
	Serial.print(" Robot.way_l= "); Serial.println(Robot.way_l);

	Serial.print(" SpeedShimR= "); Serial.print(SpeedShimR);
	Serial.print(" Robot.rpm_r= "); Serial.print(Robot.rpm_r);
	Serial.print(" Robot.speed_R= "); Serial.print(Robot.speed_R);
	Serial.print(" Robot.way_r= "); Serial.println(Robot.way_r);

}

