
package org.usfirst.frc.team177.robot;


import org.usfirst.frc.team177.auto.*;
import org.usfirst.frc.team177.lib.HTTPServer;
import org.usfirst.frc.team177.lib.IMU;
import org.usfirst.frc.team177.lib.Locator;
import org.usfirst.frc.team177.lib.Logger;

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
	
    /* Motor Controllers */
    Victor rearLeftMotor = new Victor(Constants.MotorDriveRL.getInt());
    Victor midLeftMotor = new Victor(Constants.MotorDriveML.getInt());
    Victor frontLeftMotor = new Victor(Constants.MotorDriveFL.getInt());
    
    Victor rearRightMotor = new Victor(Constants.MotorDriveRR.getInt());
    Victor midRightMotor = new Victor(Constants.MotorDriveMR.getInt()); 
    Victor frontRightMotor = new Victor(Constants.MotorDriveFR.getInt());           
    
    public RobotDrive6 drive = new RobotDrive6(frontLeftMotor, midLeftMotor, rearLeftMotor, frontRightMotor, midRightMotor, rearRightMotor);

    /* Instantiate Joysticks */
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    Joystick inputs = new Joystick(3);
    
    /* Pnumatics */ 
    Solenoid shifter = new Solenoid(Constants.SolenoidDriveShifter.getInt());
    Solenoid caster = new Solenoid(Constants.SolenoidDriveCaster.getInt());
    
    /* Automode Variables */
    int autoMode = 0;
    double autoDelay = 0;    
    AutoMode auto;
    
    /* Navigation functions */
    public Locator locator = new Locator();
    public IMU imu = new IMU();
    
    //Web server
    private Thread tHTTP;
    
    //Logger
    private Logger logger = new Logger();
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	locator.start();
    	new Thread(imu).start();
    	
    	//HTTP Server
    	tHTTP = new Thread(new HTTPServer());
    	tHTTP.start();
    	
    	//Setup Logger
    	logger.add(locator);
    	
    	/*Setup LiveWindow */        
        LiveWindow.addActuator("Left Drive", "Left Front", frontLeftMotor);
        LiveWindow.addActuator("Left Drive", "Left Mid", midLeftMotor);
        LiveWindow.addActuator("Left Drive", "Left Rear", rearLeftMotor);
        LiveWindow.addActuator("Right Drive", "Right Front", frontRightMotor);
        LiveWindow.addActuator("Right Drive", "Right Mid", midRightMotor);
        LiveWindow.addActuator("Right Drive", "Right Rear", rearRightMotor);
    }

    public void autonomousInit()
    {
    	if(auto != null) {
            auto.autoInit();
        }
    	if(logger != null) {
    		logger.start();
    	}
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() { 
        if(auto != null /*&& m_ds.getMatchTime() > autoDelay*/) {
            auto.autoPeriodic();        
        } else {
            drive.tankDrive(0, 0);
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	drive.tankDrive(leftStick, rightStick); // drive with the joysticks 
    	shifter.set(rightStick.getRawButton(Constants.shiftButton.getInt()));
    	shifter.set(leftStick.getRawButton(Constants.casterButton.getInt()));
    	
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
    
    /**
     * This function is called on entering disabled mode
     */
    public void disabledInit() {
    	logger.stop();
    }
    
    /**
     * Handle updating of automode any other pre-match diagnosticics
     */
    public void disabledPeriodic() {
    	try {
    		//int new_automode = (inputs.getRawButton(1) ? 0 : 4) + (inputs.getRawButton(2) ? 0 : 2) + (inputs.getRawButton(3) ? 0 : 1);
    		int new_automode = 2;
    		if (new_automode != autoMode)
    		{
    			autoMode = new_automode;
    			/* Add automodes here:
    			 * 0 = Do Nothing
    			 * 1 = Drive Test
    			 */
				switch (autoMode)
				{
				    case 1:
				    	auto = new AutoModeDriveTest(this);
				    	break;
				    case 2:
				    	auto = new AutoModeDriveToTest(this);
				    	break;
				    default:
				    	auto = null;
				    	break;
			    }
    		}
            autoDelay = 0; //(inputs.getX() + 1.0f)*10.0f;  //-1 to 1 gives you a range 0 - 20
        } catch (Exception e) {
            System.out.println("Error in disabledPeriodic: " + e);
        }

        //Send the selected mode to the driver station
        if(auto == null) {
            SmartDashboard.putString("Auto Mode", "Do Nothing"); 
        } else {
            SmartDashboard.putString("Auto Mode", auto.getName()); 
        }
        SmartDashboard.putNumber("Auto Delay", autoDelay);
   	}
}
