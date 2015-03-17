package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Implement interface to time of flight range finder

public class RangeFinder implements Runnable {
	
	/** Constants **/
	private final static int REG_ID 				= 0x00;
	private final static int REG_START 				= 0x18;
	private final static int REG_INTERMESASUREMENT	= 0x1D;
	private final static int REG_RANGE				= 0x62;
	private final static int REG_AVERAGING_SAMPLE_TIME	= 0x10A;
	private final static int REG_RANGE_STATUS		= 0x4D;
	
	private I2C VL6180X; 
	
	private static int VL6180X_address				= 0x52;
	private final static byte VL6180X_ID	= (byte) 0xB4;
	
	private boolean connected = false;;
	
	private byte range;
	
	public RangeFinder() {
		VL6180X = new I2C(I2C.Port.kOnboard, VL6180X_address);
		Init();
	}

	
	@Override
	public void run() {
		int cycleCnt = 0;
		while(true) {
			long startTime = System.nanoTime();
			
			//If not connected attempt to reconnect
			if(!connected) {
				Init();
			}
			//Update the gyros
			if(connected) {
				UpdateRange();
			}
			
			//Update smartdashboard at 1hz
			cycleCnt = (cycleCnt+1)%10;
			if(cycleCnt == 0) {
				SmartDashboard.putNumber("Range", get());
			}
			
			try {
				long delay = (startTime - System.nanoTime())/1000000  + 10;  //TODO make this a constant
				Thread.sleep(delay < 0 ? 1:delay ); //Update at 10ms (100Hz) 
			} catch (InterruptedException e) {
			    e.printStackTrace();
			}
		}
	}
	
	private void Init() {
		//Confirm we're talking to the device
		try {
			if(ReadReg(REG_ID) != VL6180X_ID) {
				connected = false;
				return;
			}
			
			//Configure the interter measurement time to 10ms
			if(WriteReg(REG_INTERMESASUREMENT, (byte)0)) {
				connected = false;
				return;
			}
			
			//Start continuous measurements
			if(WriteReg(REG_START, (byte)0x03)) {
				connected = false;
				return;
			}
			
			//Start continuous measurements
			if(WriteReg(REG_AVERAGING_SAMPLE_TIME, (byte)0x48)) {
				connected = false;
				return;
			}			
		} catch (Exception e) {
			System.out.println("Init: " + e);
		}
		connected = true;
	}
	
	//Read a byte using 16 bit addresses
	private byte ReadReg(int reg) {
		byte[] writeBuffer = new byte[2];
		byte[] readBuffer = new byte[1];
		
		writeBuffer[0] = (byte)(reg >> 8);
		writeBuffer[1] = (byte)(reg & 0xFF);
		
		try {
			if(VL6180X.transaction(writeBuffer, 2, readBuffer, 1)) {
				//transaction aborted
				readBuffer[0] = 0;
			}
		} catch (Exception e) {
			System.out.println("ReadReg: " + e);
		}
		return readBuffer[0];
	}

	//Write a byte using 16 bit addresses
	private boolean WriteReg(int reg, byte value) {
		byte[] writeBuffer = new byte[3];
		boolean Aborted = true;
		
		writeBuffer[0] = (byte)(reg >> 8);
		writeBuffer[1] = (byte)(reg & 0xFF);
		writeBuffer[2] = value;
		
		try {
			Aborted =  VL6180X.transaction(writeBuffer, 3, null, 0);
		} catch (Exception e) {
			System.out.println("WriteReg: " + e);
		}
		return Aborted;
	}	
	
	//Read range values
	private void UpdateRange() {
		range = ReadReg(REG_RANGE);
	}

	public double get() {
		return range;
	}
	
}


