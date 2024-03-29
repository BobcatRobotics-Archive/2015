
package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.auto.AutoMode;
import org.usfirst.frc.team177.auto.AutoMode3Totes;
import org.usfirst.frc.team177.auto.AutoModeCanAndTote;
import org.usfirst.frc.team177.auto.AutoModeCanSlide;
import org.usfirst.frc.team177.auto.AutoModeDriveForward;
import org.usfirst.frc.team177.auto.AutoModeDriveTest;
import org.usfirst.frc.team177.auto.AutoModeDriveToTest;
import org.usfirst.frc.team177.auto.AutoModePickupCan;
import org.usfirst.frc.team177.auto.AutoModeToteTunr90java;
import org.usfirst.frc.team177.lib.HTTPServer;
import org.usfirst.frc.team177.lib.Locator;
import org.usfirst.frc.team177.lib.Logger;
import org.usfirst.frc.team177.lib.RangeFinder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	
	/** IO Definitions **/
    /** Motors **/
    private static final int MotorDriveRL = 1;
    private static final int MotorDriveFL = 3;
    
    private static final int MotorDriveRR = 2;
    private static final int MotorDriveFR = 4;
    
    private static final int MotorSlide1 = 6;
    private static final int MotorSlide2 = 7;
  
    private static final int MotorCanGrabLeft = 5;
    private static final int MotorCanGrabRight = 9;
    
    private static final int MotorShoulderTilt = 8; 
    
    /** Relay Motors **/
    private static final int ClawMotor1 = 0;
    private static final int ClawMotor2 = 1;
    
    /** Joystick Constants **/ //Magic Numbers found in Joystick.class
    private static final int axisX = 0;
    private static final int axisY = 1;
       
    /** Analog IO**/
    private static final int AIShoulderPot = 1;
    
    /** Motor Controllers **/
    Victor rearLeftMotor = new Victor(MotorDriveRL);
    Victor frontLeftMotor = new Victor(MotorDriveFL);
    
    Victor rearRightMotor = new Victor(MotorDriveRR);
    Victor frontRightMotor = new Victor(MotorDriveFR); 
    
    public Victor slideMotor1 = new Victor(MotorSlide1);
    public Victor slideMotor2 = new Victor(MotorSlide2);
    
   // public Victor pickupMotor1 = new Victor(MotorPickup1);
   // public Victor pickupMotor2 = new Victor(MotorPickup2);
        
    /** Shoulder **/
    public Shoulder shoulder = new Shoulder(MotorShoulderTilt, AIShoulderPot);

    
    /** Relays **/
    public Relay clawMotor1 = new Relay(ClawMotor1, Relay.Direction.kBoth);
    public Relay clawMotor2 = new Relay(ClawMotor2, Relay.Direction.kBoth);    
    
    /** Digital Inputs **/
    public DigitalInput toteSensor = new DigitalInput(Constants.DIOToteSensor.getInt());
            
    public RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
       
    /** Instantiate Joysticks **/
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    Joystick operatorStick = new Joystick(2);
    Joystick launchpad =  new Joystick(3);
    Joystick driverController = new Joystick(4);
    
    AnalogInput range = new AnalogInput(3);
    
    /** Pneumatics **/ 
   public Solenoid clawPneumatic = new Solenoid(2);
   public Solenoid canGrabLeft = new Solenoid(1);
   public Solenoid canGrabRight = new Solenoid(3);
    
    /** State Variables **/
    boolean armLiftStateLast = false;
    boolean armLiftStateNow = false;
    
    /* Automode Variables */
    int autoMode = 0;
    double autoDelay = 0;    
    AutoMode auto;
    
    /* Navigation functions */
    public Locator locator = new Locator();
    
    //Web server
    private Thread tHTTP;
    
    //Logger
    //private Logger logger = new Logger();
    
    //Range finder
    public RangeFinder rangeFinder = new RangeFinder();
    private Thread tRangeFinder;
    
    /* Time Tracking */
    long upStartTime = 0;
    
       
    SendableChooser driveModeChooser;
    
    boolean lowArmsPickupState;
    boolean highBoxPickupState;
    boolean lifterState;
    
    enum driveModeEnum 
	{
		TankJoyStickDrive,
		SlideJoyStickDrive,
		TankControllerDrive,
		SlideControllerDrive
	};
    
    private class DriveMode 
	{
        
    	
    	private driveModeEnum mode;
    	
    	DriveMode(driveModeEnum mode) 
		{
    		this.mode = mode;
    	}
    	
    	public driveModeEnum getMode() 
		{
    		return mode;
    	}
    }
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
	{
       /* drive.setInvertedMotor(RobotDrive6.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kMidLeft, true);
        drive.setInvertedMotor(RobotDrive6.MotorType.kRearLeft, true);
        */
    	
    	locator.start();
    	
    	//HTTP Server
    	tHTTP = new Thread(new HTTPServer());
    	tHTTP.start();
    	
    	//Setup Logger
    	//logger.add(locator);
    	
    	//start rangefinder
    	tRangeFinder = new Thread(rangeFinder);
    	tRangeFinder.start();

    	/**Drive Mode Chooser **/
    	driveModeChooser = new SendableChooser();
    	for( driveModeEnum dm : driveModeEnum.values()) 
		{
    		driveModeChooser.addObject(dm.toString(), new DriveMode(dm));	
    	}
    	SmartDashboard.putData("DriveMode", driveModeChooser);
    	
    	
    	/** Setup LiveWindow **/        
        LiveWindow.addActuator("Drive", "Left Front", frontLeftMotor);
        LiveWindow.addActuator("Drive", "Left Rear", rearLeftMotor);
        LiveWindow.addActuator("Drive", "Right Front", frontRightMotor);
        LiveWindow.addActuator("Drive", "Right Rear", rearRightMotor);
    }

    
    public void autonomousInit()
    {
    	if(auto != null) 
		{
            auto.autoInit();
        }
    	/*if(logger != null) {
    		logger.clear();
    		logger.add(locator);
    		logger.add(auto);
    		logger.start();
    	}*/
    }
    
    public void teleopInit()
    {
    	/*if(logger != null) 
		{
    		logger.stop();
    	}*/
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
	{
        if(auto != null /*&& m_ds.getMatchTime() > autoDelay*/) 
		{
            auto.autoPeriodic();        
        } else 
		{
            drive.tankDrive(0, 0);
        }
    }
    
  
    
    public void disabledPeriodic() 
	{	
    	SmartDashboard.putNumber("Shoulder Position(v)", shoulder.getRawPosition());
    	SmartDashboard.putNumber("Shoulder Position(%)", shoulder.getPosition()*100.0);
    	DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
		SmartDashboard.putBoolean("Tote Sensor", toteSensor.get());
			
		try 
		{
    		int new_automode = (launchpad.getRawButton(1) ? 0 : 4) + (launchpad.getRawButton(2) ? 0 : 2) + (launchpad.getRawButton(3) ? 0 : 1);
    		if (new_automode != autoMode)
    		{
    			autoMode = new_automode;
    			//Add automodes here:
    			 
				switch (autoMode)
				{
				    case 1:
				    	auto = new AutoModeCanSlide(this);
				    	break;
				    case 2:
				    	auto = new AutoModeCanAndTote(this);
				    	break;
				    case 3:
				    	auto = new AutoModeDriveForward(this);
				    	break;
				    case 4:
				    	auto = new AutoModeToteTunr90java(this);
				    	break;
				    case 5:
				    	auto = new AutoMode3Totes(this);
				    	break;	
				    default:
				    	auto = null;
				    	break;
			    } 
    		}
            autoDelay = 0; //(inputs.getX() + 1.0f)*10.0f;  //-1 to 1 gives you a range 0 - 20
        } 
		catch (Exception e) 
		{
            System.out.println("Error in disabledPeriodic: " + e);
        }

        //Send the selected mode to the driver station
        if(auto == null) 
		{
            SmartDashboard.putString("Auto Mode", "Do Nothing"); 
        } else 
		{
            SmartDashboard.putString("Auto Mode", auto.getName()); 
        }
        SmartDashboard.putNumber("Auto Delay", autoDelay);
        
        SmartDashboard.putNumber("leftEncoder", locator.getLeftEncoderDistance());
		SmartDashboard.putNumber("Right Encoder", locator.getRightEncoderDistance());
		SmartDashboard.putNumber("Slide1 Encoder", locator.getSlide1EncoderDistance());
		SmartDashboard.putNumber("Slide2 Encoder", locator.getSlide2EncoderDistance());
		SmartDashboard.putNumber("gyro", locator.GetHeading());
   	}		

    /**
     * This function is called periodically during operator control
     */
    	
	public void teleopPeriodic() 
	{
		/** Shoulder Tilt **/
		boolean shoulder1TooLow = shoulder.getRawPosition() < (Constants.shoulderPosition1.getDouble() - Constants.shoulderDeadband.getDouble());
		boolean shoulder1TooHigh = shoulder.getRawPosition() > (Constants.shoulderPosition1.getDouble() + Constants.shoulderDeadband.getDouble());
		boolean shoulder2TooLow = shoulder.getRawPosition() < (Constants.shoulderPosition2.getDouble() - Constants.shoulderDeadband.getDouble());
		boolean shoulder2TooHigh = shoulder.getRawPosition() > Constants.shoulderPosition2.getDouble() + Constants.shoulderDeadband.getDouble();
		shoulder.set(operatorStick.getRawAxis(1) * -1); 
		clawPneumatic.set(operatorStick.getRawButton(5));
		
		/**Window Motor Control **/
		if(operatorStick.getRawButton(6)) 
		{
			clawMotor1.set(Relay.Value.kForward);
			clawMotor2.set(Relay.Value.kForward);
		} 
		else if(operatorStick.getRawButton(8)) 
		{
			clawMotor1.set(Relay.Value.kReverse);
			clawMotor2.set(Relay.Value.kReverse);
		} 
		else 
		{
			clawMotor1.set(Relay.Value.kOff);
			clawMotor2.set(Relay.Value.kOff);
		}
		if(operatorStick.getRawButton(10)) 
		{
			locator.Reset();
		}
		
		/** Smart Dashboard **/
		SmartDashboard.putNumber("Shoulder Position(v)", shoulder.getRawPosition());
    	SmartDashboard.putNumber("Shoulder Position(%)", shoulder.getPosition()*100.0);
		
		//Encoders maybe redundant here? 
		SmartDashboard.putNumber("leftEncoder", locator.getLeftEncoderDistance());
		SmartDashboard.putNumber("Right Encoder", locator.getRightEncoderDistance());
		SmartDashboard.putNumber("Slide1 Encoder", locator.getSlide1EncoderDistance());
		SmartDashboard.putNumber("Slide2 Encoder", locator.getSlide2EncoderDistance());
		
		SmartDashboard.putNumber("Range", range.getVoltage());
		
		/** Drive Mode **/
		double left = 0, right = 0, slide = 0;
		
		DriveMode ActiveDriveMode = (DriveMode) driveModeChooser.getSelected();
		
		SmartDashboard.putString("ActiveDriveMode", ActiveDriveMode.getMode().toString());
		switch (ActiveDriveMode.getMode()) {
			case TankJoyStickDrive:
				left = leftStick.getRawAxis(axisY);
				right = rightStick.getRawAxis(axisY);
				slide = rightStick.getRawAxis(axisX); 								
				break;
			case SlideControllerDrive:
				slide = driverController.getRawAxis(0);
				left = driverController.getRawAxis(1)  - driverController.getRawAxis(4);
	    		right = driverController.getRawAxis(1) + driverController.getRawAxis(4);
	    		if (left > right) 
				{
	    			if (left > 1) 
					{
	    				double scale = 1 / left;
	    				left = scale * left;
	    				right = scale * right;
	    			} 
	    		} 
				else 
				{
	    			if (right > 1) 
					{
	    				double scale = 1 / right;
	    				left = scale * left;
	    				right = scale * right;
	    			}
	    		}
				break;
			case SlideJoyStickDrive:
				slide = leftStick.getRawAxis(axisX);				
				left = leftStick.getRawAxis(axisY)  - rightStick.getRawAxis(axisX);
				right = leftStick.getRawAxis(axisY) + rightStick.getRawAxis(axisX);
				if (left > right) 
				{
					if (left > 1) 
					{
						double scale = 1 / left;
						left = scale * left;
						right = scale * right;
					} 
				} 
				else 
				{
					if (right > 1) 
					{
						double scale = 1 / right;
						left = scale * left;
						right = scale * right;
					}
				}
				break;
			case TankControllerDrive:
				left = driverController.getRawAxis(1);
				right = driverController.getRawAxis(5);
				slide = driverController.getRawAxis(0);				
				break;
			default:
				break;
		}
		//If the shift switch is set reverse the orientation of the robot
		if(!rightStick.getRawButton(3))
		{
			double temp = left;
			left = -1.0 * right;
			right = -1.0 * temp;
			slide = -1.0 * slide;
		}
		
		/* Set motor outputs in one place */		
		drive.tankDrive(left, right);
		slideMotor1.set(slide);
		slideMotor2.set(-1.0*slide);
    }  
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() 
	{
    	LiveWindow.run();
    }
}
