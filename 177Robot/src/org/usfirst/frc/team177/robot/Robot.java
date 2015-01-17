
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
	public void teleopPeriodic() {
    	
    	shifter.set(rightStick.getRawButton(shiftButton));
    	caster.set(leftStick.getRawButton(casterButton));
    	
    	/*Drive Modes
    	 * 0:Slide Drive
    	 * 1:Tank Drive
    	 */
    	final int DriveMode = 1;
    	
    	if (DriveMode == 0) {
    		if(Math.abs(leftStick.getRawAxis(axisX)) > Math.abs(leftStick.getRawAxis(axisY))) {
        		slideMotor.set(leftStick.getRawAxis(axisX));
        		drive.tankDrive(0, 0);
        	} else {
        		drive.tankDrive(leftStick.getRawAxis(axisY), leftStick.getRawAxis(axisY)); //Left, Right side
        		slideMotor.set(0);
        	} 
    	} else if (DriveMode == 1){
    		drive.tankDrive(leftStick, rightStick);
    	}
    	
    	
    	
    	SmartDashboard.putNumber("J1.X",  leftStick.getAxis(AxisType.kX));
    	SmartDashboard.putNumber("J1.Y",  leftStick.getAxis(AxisType.kY));
    	SmartDashboard.putNumber("J2.X", rightStick.getAxis(AxisType.kX));
    	SmartDashboard.putNumber("J2.Y", rightStick.getAxis(AxisType.kY));
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
