package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Implement interface to 6 Degrees of freedom IMU Module
// ADXL345 accelerometer -- Not yet fully implemented 
// ITG-3200 gyro

public class IMU implements Runnable {

	/** ITG-3200 Constants **/
	private final static byte ITG3200_address 	= (byte) 0x68;
	private final static byte ITG_REG_WHOAMI 	= (byte) 0x00;
	private final static byte ITG_REG_POWER 	= (byte) 0x3E;
	private final static byte ITG_REG_DATASTART	= (byte) 0x1D;
	private final static double ITG_DPS_SCALE = (1/14.375);  // 14.375 LSB/Degree/sec
	
	/** ADXL345 Constants **/
	private final static byte ADXL345_address 	= (byte) 0x53;
	private final static byte ADXL_REG_DEVID	= (byte) 0x0;
	private final static byte ADXL_REG_POWER_CTL= (byte) 0x2D;
	private final static byte ADXL_DEVID		= (byte) 0xE5;
	private final static byte ADXL_REG_DATASTART= (byte) 0x32;
	private final static double ADXL_SCALE =	3.9/1000.0; //LSB/mg  - +/- 2g resolution		 
	private final static double ADXL_OFFSET_SCALE =	15.6/1000.0; //LSB/mg 
	private final static byte ADXL_REG_OFFSTART= (byte) 0x1E;

	
	private double X_Acel;
	private double Y_Acel;
	private double Z_Acel;
	
	private double X_Gyro;
	private double Y_Gyro;
	private double Z_Gyro;
	
	private double Z_Angle = 0;
	
	private I2C ITGi2c; 
	private I2C ADXLi2c; 
	
	private boolean ITG_connected = false;;
	private boolean ADXL_connected = false;;
	
	public IMU() {
		ITGi2c = new I2C(I2C.Port.kOnboard, ITG3200_address);
		InitGyros();
		
		ADXLi2c = new I2C(I2C.Port.kOnboard, ADXL345_address);
		InitAccelerometers();
		CalibrateAccelerometers();
	}

	
	@Override
	public void run() {
		int cycleCnt = 0;
		while(true) {
			long startTime = System.nanoTime();
			
			//If not connected attempt to reconnect
			if(!ITG_connected) {
				InitGyros();
			}
			//Update the gyros
			if(ITG_connected) {
				UpdateGyros();
				Z_Angle += Z_Gyro/100.0; //Z_Gyro is in degrees per second
			}
			
			//If not connected attempt to reconnect
			if(!ADXL_connected) {
				InitAccelerometers();
			}
			//Update the Accelerometers
			if(ADXL_connected) {
				UpdateAccelerometers();
			}
			
			//Update the sensor fusion calculations
			MadgwickAHRSupdateIMU((float)X_Gyro, (float)Y_Gyro, (float)Z_Gyro, (float)X_Acel, (float)Y_Acel, (float)Z_Acel);
			
			//Update smartdashboard at 1hz
			cycleCnt = (cycleCnt+1)%10;
			if(cycleCnt == 0) {
				SmartDashboard.putNumber("X IMU", getXAngle());
				SmartDashboard.putNumber("Y IMU", getYAngle());
				SmartDashboard.putNumber("Z IMU", getZAngle());
				SmartDashboard.putNumber("Z Angle", Z_Angle);
			}
			
			try {
				long delay = (startTime - System.nanoTime())/1000000  + 10;  //TODO make this a constant
				Thread.sleep(delay < 0 ? 1:delay ); //Update the gyro at 10ms (100Hz) 
			} catch (InterruptedException e) {
			    e.printStackTrace();
			}
		}
	}
	
	private void InitGyros() {
		byte[] readBuffer = new byte[1];
		
		//Confirm we're talking to the device
		try {
			if(!ITGi2c.read(ITG_REG_WHOAMI, 1, readBuffer) || readBuffer[0] != ITG3200_address) {
				ITG_connected = false;
				return;
			}
			
			//Configure the clock source to be the Z-Gyro
		    if(ITGi2c.write(ITG_REG_POWER, 0x03)) {
				ITG_connected = false;
				return;
			}
		    
		} catch (Exception e) {
			System.out.println("InitGyros: " + e);
		}
		ITG_connected = true;
	}
	
	//Read gyro values
	private void UpdateGyros() {
		byte[] rawData = new byte[6];
		
		try {
			//Read all the gyros at once
			if(ITGi2c.read(ITG_REG_DATASTART, 6, rawData)) {
				//Extract the registers
				X_Gyro = ((rawData[0] << 8) + rawData[1])*ITG_DPS_SCALE;
				Y_Gyro = ((rawData[2] << 8) + rawData[3])*ITG_DPS_SCALE;
				Z_Gyro = ((rawData[4] << 8) + rawData[5])*ITG_DPS_SCALE;
			} else {
				X_Gyro = Y_Gyro = Z_Gyro = 0;
			}
		} catch (Exception e) {
			System.out.println("UpdateAccelerometers: " + e);
		}
		
		SmartDashboard.putNumber("X Gyro", X_Gyro);
		SmartDashboard.putNumber("Y Gyro", Y_Gyro);
		SmartDashboard.putNumber("Z Gyro", Z_Gyro);
	}

	public double getX_Gyro() {
		return X_Gyro;
	}
	
	public double getY_Gyro() {
		return Y_Gyro;
	}
	
	public double getZ_Gyro() {
		return Z_Gyro;
	}
	
	public double GetHeading() {
		//Wrapper to make this compatible with the default gyro
		return (double) getZ_Gyro();
	}

	//calibrate and set offset registers. 
	//This must be performed with the device motionless and oriented correctly
	private void CalibrateAccelerometers() {
		int samplecnt = 0;
		double x = 0;
		double y = 0;
		double z = 0;
		
		if(!ADXL_connected) {
			return;
		}
		
		//Reset the offset registers
		//Scale to offset register
		byte[] writeBuffer = new byte[4];
		writeBuffer[0] = ADXL_REG_OFFSTART;
		writeBuffer[1] = 0;
		writeBuffer[2] = 0;
		writeBuffer[3] = 0;
				
		//Write to the offset register
		ADXLi2c.writeBulk(writeBuffer);
		
		//Delay to give the accelerometer time to startup and settle
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		//Take 10 samples and average them
		for(samplecnt = 0; samplecnt < 10; samplecnt++) {
			UpdateAccelerometers();
			x += X_Acel;
			y += Y_Acel;
			z += Z_Acel;
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		//calculate average
		x /= 10;
		y /= 10;
		z /= 10;
		System.out.println(String.format("Accel Calibration %.2f %.2f %.2f", x, y, z));
		
		//Scale to offset register
		writeBuffer[0] = ADXL_REG_OFFSTART;
		writeBuffer[1] = (byte) (-x / ADXL_OFFSET_SCALE);
		writeBuffer[2] = (byte) (-y / ADXL_OFFSET_SCALE);
		writeBuffer[3] = (byte) (-z / ADXL_OFFSET_SCALE);
		
		//Write to the offset register
		ADXLi2c.writeBulk(writeBuffer);
		
	}

	private void InitAccelerometers() {
		byte[] readBuffer = new byte[1];
		
		//Confirm we're talking to the device
		try {
			if(!ADXLi2c.read(ADXL_REG_DEVID, 1, readBuffer) || readBuffer[0] != ADXL_DEVID) {
				ADXL_connected = false;
				return;
			}
			
			//Enable Measurement
		    if(ADXLi2c.write(ADXL_REG_POWER_CTL, 0x08)) {
		    	ADXL_connected = false;
				return;
			}
		} catch (Exception e) {
			System.out.println("InitAccelerometers: " + e);
		}
		ADXL_connected = true;
	}
	
	//Read Accelerometers values
	private void UpdateAccelerometers() {
		byte[] rawData = new byte[6];
		
		try {
			//Read all the gyros at once
			if(ADXLi2c.read(ADXL_REG_DATASTART, 6, rawData)) {
				//Extract the registers
				X_Acel = ((rawData[1] << 8) + rawData[0])*ADXL_SCALE;
				Y_Acel = ((rawData[3] << 8) + rawData[2])*ADXL_SCALE;
				Z_Acel = ((rawData[5] << 8) + rawData[4])*ADXL_SCALE;
			} else {
				X_Acel = Y_Acel = Z_Acel = 0;
			}
		} catch (Exception e) {
			System.out.println("UpdateAccelerometers: " + e);
		}
			
		SmartDashboard.putNumber("X Acel", X_Acel);
		SmartDashboard.putNumber("Y Acel", Y_Acel);
		SmartDashboard.putNumber("Z Acel", Z_Acel);
	}
	
	
	
	//Adapted from:
	//Implementation of Madgwick's IMU and AHRS algorithms.
	//See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

	//Other possible references
	// http://www.chrobotics.com/library/understanding-quaternions
	// https://code.google.com/p/nav6/source/browse/trunk/arduino/nav6/nav6.ino
	// http://www.cprogramming.com/tutorial/3d/quaternions.html
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
	// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation

	//---------------------------------------------------------------------------------------------------
	// Definitions

	static final float sampleFreq =	512.0f;		// sample frequency in Hz
	static final float betaDef =	0.1f;		// 2 * proportional gain

	//---------------------------------------------------------------------------------------------------
	// Variable definitions

	volatile float beta = betaDef;								// 2 * proportional gain (Kp)
	volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

	//http://www.chrobotics.com/library/understanding-quaternions	
	public double getXAngle() { //Roll
		return Math.atan2((2*(q0*q1 + q2*q3)),(q0*q0 - q1*q1 - q2*q2 + q3*q3));
	}
	
	public double getYAngle() { //Pitch
		return -1*Math.asin(2*(q1*q3 + q0*q2));
	}
	
	public double getZAngle() { //Yaw
		return Math.atan2((2*(q0*q3 + q1*q2)),(q0*q0 + q1*q1 - q2*q2 - q3*q3));
	}

	//---------------------------------------------------------------------------------------------------
	// IMU algorithm update
	private void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_4q0 = 4.0f * q0;
			_4q1 = 4.0f * q1;
			_4q2 = 4.0f * q2;
			_8q1 = 8.0f * q1;
			_8q2 = 8.0f * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0f / sampleFreq);
		q1 += qDot2 * (1.0f / sampleFreq);
		q2 += qDot3 * (1.0f / sampleFreq);
		q3 += qDot4 * (1.0f / sampleFreq);

		//TODO - consider adding a check to see if normalization is necessary (sum of squares close to 1 = not necisary)
		// Normalise quaternion - make square root of the sum of squares = 1 
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}

	//---------------------------------------------------------------------------------------------------
	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

	public static float invSqrt(float x) {
	    float xhalf = 0.5f*x;
	    int i = Float.floatToIntBits(x);
	    i = 0x5f3759df - (i>>1);
	    x = Float.intBitsToFloat(i);
	    x = x*(1.5f - xhalf*x*x);
	    return x;
	}
	
	public static double invSqrt(double x) {
	    double xhalf = 0.5d*x;
	    long i = Double.doubleToLongBits(x);
	    i = 0x5fe6ec85e7de30daL - (i>>1);
	    x = Double.longBitsToDouble(i);
	    x = x*(1.5d - xhalf*x*x);
	    return x;
	}

}


