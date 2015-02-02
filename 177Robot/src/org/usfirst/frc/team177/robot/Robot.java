
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
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
    /* Motors */
    private static final int MotorDriveRL = 1;
    private static final int MotorDriveFL = 3;
    
    private static final int MotorDriveRR = 2;
    private static final int MotorDriveFR = 4;
    
    private static final int MotorSlide = 7;
    
    private static final int MotorStack = 6;
    
    /* Joystick Constants */ //Magic Numbers found in Joystick.class
    private static final int axisX = 0;
    private static final int axisY = 1;
    
    /**Controller Constants**/
    private static final int ControllerStackButton0 = 1;
    private static final int ControllerStackButton1 = 2;
    private static final int ControllerStackButton2 = 3;
    
    /** Digital IO */
    private static final int DIOLeftEncoderA = 0;
    private static final int DIOLeftEncoderB = 1;
    private static final int DIORightEncoderA = 2;
    private static final int DIORightEncoderB = 3;
    
    /** Right Joystick Buttons **/
    

    /** Left Joystick Buttons **/
    
    /* Motor Controllers */
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor frontLeftMotor = new Victor(MotorDriveFL);
    
    Victor rearRightMotor = new Victor(MotorDriveRR);
    Victor frontRightMotor = new Victor(MotorDriveFR); 
    
    Victor slideMotor = new Victor(MotorSlide);
    
    Talon stackMotor = new Talon(MotorStack);
    
    RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    
    /* Encoders */
    Encoder leftEncoder = new Encoder(DIOLeftEncoderA,DIOLeftEncoderB);
    Encoder rightEncoder = new Encoder(DIORightEncoderA,DIORightEncoderB);
    
    /* Instantiate Joysticks */
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    Joystick launchpad =  new Joystick(3);
    
    /* Pneumatics */ 
    Solenoid stacker0 = new Solenoid(0);
    Solenoid stacker1 = new Solenoid(1);
    Solenoid stacker2 = new Solenoid(2);
    
    SendableChooser driveModeChooser;
    
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
    	leftEncoder.setDistancePerPulse(1);
    	rightEncoder.setDistancePerPulse(1);
    	driveModeChooser = new SendableChooser();
    	for( driveModeEnum dm : driveModeEnum.values()) {
    		driveModeChooser.addObject(dm.toString(), new DriveMode(dm));
    		
    	}
    	SmartDashboard.putData("DriveMode", driveModeChooser);
    	
    	
    	/*Setup LiveWindow */        
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
    	DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
    }
    

    /**
     * This function is called periodically during operator control
     */
    @SuppressWarnings("unused")
    /*
     * Switches: Left Right Middle.  1 is up, 0 is down
     * 1 2 3
     * 
     * 0 0 0 Joystick Tank Drive
     * 0 0 1 Controller Tank Drive
     * 1 0 0 Joystick Slide Drive
     * 1 0 1 Controller Slide Drive
     */
	public void teleopPeriodic() {
    	stacker0.set(operatorStick.getRawButton(1));
		stacker1.set(operatorStick.getRawButton(1));
		stacker2.set(operatorStick.getRawButton(2));
		double left, right;
		
		stackMotor.set(operatorStick.getRawAxis(axisY));
		
		SmartDashboard.putNumber("Operator YAxis", operatorStick.getRawAxis(axisY));
		SmartDashboard.putNumber("leftEncoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());

		slideMotor.set(leftStick.getRawAxis(axisX) * -1);
		DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
		switch (ActiveDriveMode.getMode()) {
			case TankJoyStickDrive:
				drive.tankDrive(leftStick, rightStick);
				break;
			case SlideControllerDrive:
				slideMotor.set(operatorStick.getRawAxis(0));
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
				break;
			default:
				break;
		}
		
		/*
		//caster.set(leftStick.getRawButton(casterButton));
		//shifter.set(rightStick.getRawButton(shiftButton));
		//drive.tankDrive(leftStick, rightStick);
    	if (launchpad.getRawButton(leftSwitch) == false && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == false) {  //Joystick Tank Drive
    		
    		SmartDashboard.putString("Mode", "Tank Drive");
    		stacker.set(leftStick.getRawButton(StackerButton));
    		slider.set(rightStick.getRawButton(SliderButton));
    		drive.tankDrive(leftStick, rightStick);
    	
    	} else if (launchpad.getRawButton(leftSwitch) == false && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == true) {  //Controller Tank Drive
    		slider.set(operatorStick.getRawButton(ControllerShiftButton));
    		stacker.set(operatorStick.getRawButton(ControllerCasterButton));
    		drive.tankDrive(operatorStick.getRawAxis(1), operatorStick.getRawAxis(3));
    	} else if (launchpad.getRawButton(leftSwitch) == true && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == false) {  //Joystick Slide Drive
    		stacker.set(leftStick.getRawButton(StackerButton));
    		slider.set(rightStick.getRawButton(SliderButton)); 
    		double left = leftStick.getRawAxis(axisY)  - rightStick.getRawAxis(axisX);
    		double right = leftStick.getRawAxis(axisY) + rightStick.getRawAxis(axisX);
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
    		slideMotor.set(leftStick.getRawAxis(axisX));
    	} else if (launchpad.getRawButton(leftSwitch) == true && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == true) {  //Controller Slide Drive
    		slider.set(operatorStick.getRawButton(ControllerShiftButton));
    		stacker.set(operatorStick.getRawButton(ControllerCasterButton));
    		double left = operatorStick.getRawAxis(1)  - operatorStick.getRawAxis(2);
    		double right = operatorStick.getRawAxis(1) + operatorStick.getRawAxis(2
    				);
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
        	slideMotor.set(leftStick.getRawAxis(axisX));
    	} else {
    		stacker.set(leftStick.getRawButton(StackerButton));
    		slider.set(rightStick.getRawButton(SliderButton));
    		drive.tankDrive(leftStick, rightStick);
    	}
    	*/
    }  
    /**
     * This function is called periodically during test mode
     */
    	
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
