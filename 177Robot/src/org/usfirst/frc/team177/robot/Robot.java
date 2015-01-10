
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    /** Right Joystick Buttons **/
    private static final int shiftButton = 3; //Right Joystick button 3 is the shifter

    /** Left Joystick Buttons **/
    
    /* Solenoids */
    private static final int SolenoidDriveShifter = 0;
    
    /* Motor Controllers */
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor midLeftMotor = new Victor(MotorDriveML);
    Victor frontLeftMotor = new Victor(MotorDriveFL);
    
    Victor rearRightMotor = new Victor(MotorDriveRR);
    Victor midRightMotor = new Victor(MotorDriveMR); 
    Victor frontRightMotor = new Victor(MotorDriveFR);           
    
    RobotDrive6 drive = new RobotDrive6(frontLeftMotor, midLeftMotor, rearLeftMotor, frontRightMotor, midRightMotor, rearRightMotor);

    /* Instantiate Joysticks */
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    
    /* Pnumatics */ 
    Solenoid shifter = new Solenoid(SolenoidDriveShifter);
    
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
    public void teleopPeriodic() {
    	drive.tankDrive(leftStick, rightStick); // drive with the joysticks 
    	shifter.set(rightStick.getRawButton(shiftButton));
    	
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
