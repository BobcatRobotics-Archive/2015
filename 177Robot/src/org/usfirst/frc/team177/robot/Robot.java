
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
    private static final int MotorDriveML = 3;
    private static final int MotorDriveFL = 5;
    
    private static final int MotorDriveRR = 2;
    private static final int MotorDriveMR = 4;
    private static final int MotorDriveFR = 6;
    
    private static final int MotorSlide = 7;
    
    /* Joystick Constants */ //Magic Numbers found in Joystick.class
    private static final int axisX = 0;
    private static final int axisY = 1;
    
    /** Digital IO */
    private static final int DIOLeftEncoderA = 0;
    private static final int DIOLeftEncoderB = 1;
    private static final int DIORightEncoderA = 2;
    private static final int DIORightEncoderB = 3;
    /** Right Joystick Buttons **/
    private static final int shiftButton = 3; //Right Joystick button 3 is the shifter

    /** Left Joystick Buttons **/
    private static final int casterButton = 3; //Left Joystick button 3 is the caster 
    
    private static final int SolenoidDriveShifter = 0;
    private static final int SolenoidCasters = 1;
    
    /** Launchpad Switches **/
    private static final int leftSwitch = 13;
    private static final int middleSwitch = 20;
    private static final int rightSwitch = 12;
    
    /* Motor Controllers */
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor midLeftMotor = new Victor(MotorDriveML);
    Victor frontLeftMotor = new Victor(MotorDriveFL);
    
    Victor rearRightMotor = new Victor(MotorDriveRR);
    Victor midRightMotor = new Victor(MotorDriveMR); 
    Victor frontRightMotor = new Victor(MotorDriveFR);
    
    Victor slideMotor = new Victor(MotorSlide);
    
    RobotDrive6 drive = new RobotDrive6(frontLeftMotor, midLeftMotor, rearLeftMotor, frontRightMotor, midRightMotor, rearRightMotor);
    
    /* Instantiate Joysticks */
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    Joystick launchpad =  new Joystick(3);
    
    /* Pneumatics */ 
    Solenoid shifter = new Solenoid(SolenoidDriveShifter);
    Solenoid caster = new Solenoid(SolenoidCasters);
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
       /* drive.setInvertedMotor(RobotDrive6.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kMidLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kRearLeft, true);
        */
    	
    	/*Setup LiveWindow */        
        LiveWindow.addActuator("Drive", "Left Front", frontLeftMotor);
        LiveWindow.addActuator("Drive", "Left Mid", midLeftMotor);
        LiveWindow.addActuator("Drive", "Left Rear", rearLeftMotor);
        LiveWindow.addActuator("Drive", "Right Front", frontRightMotor);
        LiveWindow.addActuator("Drive", "Right Mid", midRightMotor);
        LiveWindow.addActuator("Drive", "Right Rear", rearRightMotor);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
 
    }

    /**
     * This function is called periodically during operator control
     */
    @SuppressWarnings("unused")
    /*
     * Switches: Left Right Middle.  1 is up, 0 is down
     * 1 2 3
     * 
     * 0 0 0 Joystick Slide Drive
     * 0 0 1 Controller Slide Drive
     * 1 0 0 Joystick Tank Drive
     * 1 0 1 Controller Tank Drive
     */
	public void teleopPeriodic() {
    	
    	caster.set(leftStick.getRawButton(casterButton));
    	
    	
    	if (launchpad.getRawButton(leftSwitch) == false && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == false) {  //Joystick Slide Drive
    		shifter.set(rightStick.getRawButton(shiftButton));
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
    	} else if (launchpad.getRawButton(leftSwitch) == false && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == true) {  //Controller Slide Drive
    		shifter.set(operatorStick.getRawButton(6));
    		double left = operatorStick.getRawAxis(1)  - operatorStick.getRawAxis(2);
    		double right = operatorStick.getRawAxis(1) + operatorStick.getRawAxis(2);
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
    	} else if (launchpad.getRawButton(leftSwitch) == true && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == false) {  //Joystick Tank Drive
    		shifter.set(rightStick.getRawButton(shiftButton));
    		drive.tankDrive(leftStick, rightStick);
    	} else if (launchpad.getRawButton(leftSwitch) == true && launchpad.getRawButton(middleSwitch) == false && launchpad.getRawButton(rightSwitch) == true) {  //Controller Tank Drive
    		shifter.set(operatorStick.getRawButton(6));
    		drive.tankDrive(operatorStick.getRawAxis(1), operatorStick.getRawAxis(3));
    	} else {
    		drive.tankDrive(leftStick, rightStick);
    	}
    } 
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
