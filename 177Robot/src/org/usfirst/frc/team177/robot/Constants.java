package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.lib.*;

public class Constants extends ConstantsBase {

	/** IO Definitions **/
    /* Motors */
	public static final Constant MotorDriveRL = new Constant("MotorDriveRL", 1);
	public static final Constant MotorDriveML = new Constant("MotorDriveML", 3);
	public static final Constant MotorDriveFL = new Constant("MotorDriveFL", 5);
	
	public static final Constant MotorDriveRR = new Constant("MotorDriveRR", 2);
	public static final Constant MotorDriveMR = new Constant("MotorDriveMR", 4);
	public static final Constant MotorDriveFR = new Constant("MotorDriveFR", 6);
	
    /** Right Joystick Buttons **/
	public static final Constant shiftButton = new Constant("shiftButton", 3); //Right Joystick button 3 is the shifter

    /** Left Joystick Buttons **/
    public static final Constant casterButton = new Constant("casterButton", 3); //Left Joystick button 3 is the caster
    
    /** Solenoids **/
    public static final Constant SolenoidDriveShifter = new Constant("SolenoidDriveShifter", 0);
    public static final Constant SolenoidDriveCaster = new Constant("SolenoidDriveCaster", 1);
    
    /** Digital IO **/
    public static final Constant DIOLeftEncoderA = new Constant("DIOLeftEncoderA", 0);
    public static final Constant DIOLeftEncoderB = new Constant("DIOLeftEncoderB", 1);    
    public static final Constant DIORightEncoderA = new Constant("DIORightEncoderA", 2);
    public static final Constant DIORightEncoderB = new Constant("DIORightEncoderB", 3);
    
    
    /** Encoder Distance Per Pulse **/
    public static final Constant LeftEncoderDPP = new Constant("LeftEncoderDPP", 0.086128257f);
    public static final Constant RightEncoderDPP = new Constant("RightEncoderDPP", 0.086128257f);
    
    /** Analog Inputs **/
    public static final Constant AIOGyro = new Constant("AIOGyro", 0);
    
    /** Trajectory config **/
    public static final Constant maxVel = new Constant("maxVel", 15.0);
    public static final Constant maxAcc = new Constant("maxAcc", 16.0);
    public static final Constant maxJerk = new Constant("maxJerk", 15.0 * 5);
    public static final Constant updateDt = new Constant("updateDT", 10);
    
    public static final Constant trajKp = new Constant("trajKp", 1.5);
    public static final Constant trajKi = new Constant("trajKi", 0.0);
    public static final Constant trajKd = new Constant("trajKd", 0.0);
    public static final Constant trajKv = new Constant("trajKv", 1.0/15.0);
    public static final Constant trajKa = new Constant("trajKa", 1.0/34.0);
    public static final Constant kTurn = new Constant("kTurn", -3.0/80.0);
    
    public static final Constant robotWidth = new Constant("RobotWidth", 28);
    
    /** Logging **/
    public static final Constant loggingDt = new Constant("LoggingDt", 100);
    
}
