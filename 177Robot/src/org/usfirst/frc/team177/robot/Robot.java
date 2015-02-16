
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.hal.RelayJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	/** IO Definitions **/
    /** Motors **/
    private static final int MotorDriveRL = 1;
    private static final int MotorDriveFL = 3;
    
    private static final int MotorDriveRR = 2;
    private static final int MotorDriveFR = 4;
    
    private static final int MotorSlide1 = 6;
    private static final int MotorSlide2 = 7;
  
    private static final int MotorPickup = 5;
    
    private static final int MotorShoulderTilt = 8;
    /** Relay Motors **/
    private static final int MotorWindow1 = 0;
    private static final int MotorWindow2 = 1;
    
    /** Joystick Constants **/ //Magic Numbers found in Joystick.class
    private static final int axisX = 0;
    private static final int axisY = 1;
    
    /** Digital IO */
    private static final int DIOLeftEncoderA = 0;
    private static final int DIOLeftEncoderB = 1;
    private static final int DIORightEncoderA = 2;
    private static final int DIORightEncoderB = 3;
    
    /** Analog IO**/
    private static final int AIShoulderPot = 1;
    
    /** Motor Controllers **/
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor frontLeftMotor = new Victor(MotorDriveFL);
    
    Victor rearRightMotor = new Victor(MotorDriveRR);
    Victor frontRightMotor = new Victor(MotorDriveFR); 
    
    Victor slideMotor1 = new Victor(MotorSlide1);
    Victor slideMotor2 = new Victor(MotorSlide2);
    
    Talon pickupMotor = new Talon(MotorPickup);
    
    Victor shoulderTiltMotor = new Victor(MotorShoulderTilt);
    
    Relay window1 = new Relay(MotorWindow1);
    Relay window2 = new Relay(MotorWindow2);
    
    RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    
    /** Encoders **/
    Encoder leftEncoder = new Encoder(DIOLeftEncoderA,DIOLeftEncoderB);
    Encoder rightEncoder = new Encoder(DIORightEncoderA,DIORightEncoderB);
    
    /** Analog Input **/
    AnalogInput shoulderPosition = new AnalogInput(AIShoulderPot);
    
    /** Instantiate Joysticks **/
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    Joystick launchpad =  new Joystick(3);
    
    /** Pneumatics **/ 
    Solenoid lifter = new Solenoid(0);
    Solenoid lowArmsPickup = new Solenoid(1);
    Solenoid highBoxPickup = new Solenoid(2);
    Solenoid shoulderTiltPneumatic = new Solenoid(3);
    
    SendableChooser driveModeChooser;
    
    boolean lowArmsPickupState;
    boolean highBoxPickupState;
    boolean lifterState;
    
    enum driveModeEnum {
		TankJoyStickDrive,
		SlideJoyStickDrive,
		TankControllerDrive,
		SlideControllerDrive
	};
    
    private class DriveMode {
        
    	
    	private driveModeEnum mode;
    	
    	DriveMode(driveModeEnum mode) {
    		this.mode = mode;
    	}
    	
    	public driveModeEnum getMode() {
    		return mode;
    	}
    }
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
       /* drive.setInvertedMotor(RobotDrive6.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kMidLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kRearLeft, true);
        */
    	
    	/** Window Motor Setup **/
    	window1.setDirection(Relay.Direction.kBoth);
    	window2.setDirection(Relay.Direction.kBoth);
    	
    	/**Encoder Setup **/
    	leftEncoder.setDistancePerPulse(1);
    	rightEncoder.setDistancePerPulse(1);
    	
    	/**Drive Mode Chooser **/
    	driveModeChooser = new SendableChooser();
    	for( driveModeEnum dm : driveModeEnum.values()) {
    		driveModeChooser.addObject(dm.toString(), new DriveMode(dm));
    		
    	}
    	SmartDashboard.putData("DriveMode", driveModeChooser);
    	
    	
    	/** Setup LiveWindow **/        
        LiveWindow.addActuator("Drive", "Left Front", frontLeftMotor);
        LiveWindow.addActuator("Drive", "Left Rear", rearLeftMotor);
        LiveWindow.addActuator("Drive", "Right Front", frontRightMotor);
        LiveWindow.addActuator("Drive", "Right Rear", rearRightMotor);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
 
    }
    
    public void disabledPeriodic() {
    	SmartDashboard.putNumber("Shoulder Position", shoulderPosition.getVoltage());
    	DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
    }
    

    /**
     * This function is called periodically during operator control
     */
	public void teleopPeriodic() {
		/** Shoulder Tilt **/
		shoulderTiltMotor.set(operatorStick.getRawAxis(3));   //Untested might not work
		shoulderTiltPneumatic.set(operatorStick.getRawButton(4));    //Untested might not work
		
		/**Window Motor Control **/
		if(operatorStick.getRawButton(6)) {
			window1.set(Relay.Value.kForward);
			window2.set(Relay.Value.kForward);
		} else if(operatorStick.getRawButton(8)) {
			window1.set(Relay.Value.kReverse);
			window2.set(Relay.Value.kReverse);
		} else {
			window1.set(Relay.Value.kOff);
			window2.set(Relay.Value.kOff);
		}
		
		/** Stacking Controller bindings **/
		pickupMotor.set(operatorStick.getRawAxis(1));
		if (operatorStick.getRawAxis(5) > 0) {     //There is a high chance this is wrong
			lowArmsPickupState = true;
		}
		lifterState = operatorStick.getRawButton(5);
		if (operatorStick.getRawAxis(6) > 0) {     //There is a high chance this is wrong
			highBoxPickupState = true;
		}
		
		/** Stacker Anti-Failure Logic **/
		lifter.set(lifterState);
		lowArmsPickup.set(lowArmsPickupState);
		if(!lowArmsPickupState) {
		 	highBoxPickup.set(highBoxPickupState);
		}
		
		/** Smart Dashboard **/
		SmartDashboard.putNumber("Shoulder Position", shoulderPosition.getVoltage());
		SmartDashboard.putNumber("Operator YAxis", operatorStick.getRawAxis(axisY));
		SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
		
		/** Drive Mode **/
		double left , right;
		
		DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
		switch (ActiveDriveMode.getMode()) {
			case TankJoyStickDrive:
				drive.tankDrive(leftStick, rightStick);
				
				break;
			case SlideControllerDrive:
				slideMotor1.set(operatorStick.getRawAxis(0));
				slideMotor2.set(operatorStick.getRawAxis(0) * -1);
				left = operatorStick.getRawAxis(1)  - operatorStick.getRawAxis(2);
	    		right = operatorStick.getRawAxis(1) + operatorStick.getRawAxis(2);
	    		if (left > right) {
	    			if (left > 1) {
	    				double scale = 1 / left;
	    				left = scale * left;
	    				right = scale * right;
	    			} 
	    		} else {
	    			if (right > 1) {
	    				double scale = 1 / right;
	    				left = scale * left;
	    				right = scale * right;
	    			}
	    		}
	        	drive.tankDrive(left, right);
				break;
			case SlideJoyStickDrive:
				slideMotor1.set(leftStick.getRawAxis(axisX));
				slideMotor2.set(leftStick.getRawAxis(axisX) * -1);
				left = leftStick.getRawAxis(axisY)  - rightStick.getRawAxis(axisX);
				right = leftStick.getRawAxis(axisY) + rightStick.getRawAxis(axisX);
				if (left > right) {
					if (left > 1) {
						double scale = 1 / left;
						left = scale * left;
						right = scale * right;
					} 
				} else {
					if (right > 1) {
						double scale = 1 / right;
						left = scale * left;
						right = scale * right;
					}
				}

				drive.tankDrive(left, right);
				break;
			case TankControllerDrive:
				drive.tankDrive(operatorStick.getRawAxis(1), operatorStick.getRawAxis(3));
				slideMotor1.set(operatorStick.getRawAxis(0));
				slideMotor2.set(leftStick.getRawAxis(0) * -1);
				break;
			default:
				break;
		}
    }  
    /**
     * This function is called periodically during test mode
     */
    	
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
