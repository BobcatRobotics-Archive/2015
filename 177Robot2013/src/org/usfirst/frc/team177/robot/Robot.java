
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
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
    private static final int MotorDriveRR = 2;
    private static final int MotorDriveFL = 3;
    private static final int MotorDriveFR = 4;  
    
    /* Motor Controllers */
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor rearRightMotor = new Victor(MotorDriveRR);

    Victor frontLeftMotor = new Victor(MotorDriveFL);
    Victor frontRightMotor = new Victor(MotorDriveFR);           
    
    RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor,frontRightMotor, rearRightMotor);
    
    /* Instantiate Joysticks */
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    Joystick operatorStick = new Joystick(3);
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
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

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	drive.tankDrive(leftStick, rightStick); // drive with the joysticks 
    	
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
