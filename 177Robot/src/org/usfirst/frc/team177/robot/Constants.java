package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.lib.*;

public class Constants extends ConstantsBase {

	/** Shoulder Config **/
	public static final Constant shoulderMinV = new Constant("shoulderMinV", 0.72);
	public static final Constant shoulderMaxV = new Constant("shoulderMaxV", 2.98);
	public static final Constant shoulderLimitUp = new Constant("shoulderLimitUp", 0.72);
	public static final Constant shoulderLimitDown = new Constant("shoulderLimitDown", 2.98);
	public static final Constant shoulderEnableLimits = new Constant("shoulderEnableLimits", 1);
	public static final Constant shoulderDeadband = new Constant("shoulderDeadband", 0.0);
	
	/** Encoders **/
	public static final Constant DIOLeftEncoderA = new Constant("DIOLeftEncoderA", 0);
	public static final Constant DIOLeftEncoderB = new Constant("DIOLeftEncoderB", 1);
	public static final Constant DIORightEncoderA = new Constant("DIORightEncoderA", 2);
	public static final Constant DIORightEncoderB = new Constant("DIORightEncoderB", 3);
	public static final Constant DIOSlide1EncoderA = new Constant("DIOSlide1EncoderA", 4);
	public static final Constant DIOSlide1EncoderB = new Constant("DIOSlide1EncoderB", 5);
	public static final Constant DIOSlide2EncoderA = new Constant("DIOSlide2EncoderA", 6);
	public static final Constant DIOSlide2EncoderB = new Constant("DIOSlide2EncoderB", 7);
	
	/** Encoder Calibration */
	public static final Constant LeftEncoderDPP = new Constant("LeftEncoderDPP", 12.57/128);
	public static final Constant RightEncoderDPP = new Constant("RightEncoderDPP", 12.57/128);
	public static final Constant SlideEncoderDPP = new Constant("SlideEncoderDPP", 12.57/128);
	
	/** Digital Inputs **/
	public static final Constant DIOToteSensor = new Constant("DIOToteSensor", 8);
	
	/** Analog inputs **/
	public static final Constant AIOGyro = new Constant("AIOGyro", 0);	
	
    /** Trajectory config **/
    public static final Constant maxVel = new Constant("maxVel", 15.0);
    public static final Constant maxAcc = new Constant("maxAcc", 16.0);
    public static final Constant maxJerk = new Constant("maxJerk", 15.0 * 5);
    public static final Constant updateDt = new Constant("updateDT", 10);
    
    public static final Constant trajKp = new Constant("trajKp", 0.1);
    public static final Constant trajKi = new Constant("trajKi", 0.0);
    public static final Constant trajKd = new Constant("trajKd", 0.0);
    public static final Constant trajKv = new Constant("trajKv", 1.0/15.0);
    public static final Constant trajKa = new Constant("trajKa", 1.0/34.0);
    public static final Constant kTurn = new Constant("kTurn", -3.0/80.0);
    
    public static final Constant robotWidth = new Constant("RobotWidth", 41.5);
    public static final Constant robotLength = new Constant("RobotLength", 27.5);
    
    /** Logging **/
    public static final Constant loggingDt = new Constant("LoggingDt", 100);
    
    /** Time Keeping **/
    public static final Constant holderTimeOut = new Constant("Holder Time Out", 500);
    
}
