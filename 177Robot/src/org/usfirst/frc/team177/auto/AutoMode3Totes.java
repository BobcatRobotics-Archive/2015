/*
 * Basic Automode to test frame work, drive forward for 3 feet then turn 90 degrees to right
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author schroed
 */
public class AutoMode3Totes extends AutoMode {
    
	private enum steps {
    	STEP_SLIDE,
    	STEP_DRIVE_CLOSE,
    	STEP_SLIDE_IN,
    	STEP_DRIVE_TO_SENSOR,
    	STEP_PUT_THINGS_DOWN,
    	STEP_STEP_FWD,
    	STEP_SUCK_IN,
    	STEP_SLIDE_TO_AUTOZONE,
    	STEP_PUT_DOWN_IN_AUTOZONE,
    	STEP_BACK_AWAY_SLOWLY,
    	STEP_DONE
    };
    
    private steps currentStep = steps.STEP_SLIDE;
    private long lastEventTime = 0;;
    private int ToteCount = 0;
    private double upAngles[] = { 1.44, 2} ;   //Guese
    private int toteDistances[] = {80-22-3, 160-22-3}; 
    private float canLowerDistance = 10; 	//cm //gueses?
    private float downAngle = 0.9f;
    private float suckInTime = 200;	//ms
    
    private static final float slideDistance = -18;   //distance to move to the side
    private static final float slideDistanceToAuto = -180;   //distance to move on the final side
    
    private static final float shoulderUpSpeed = 0.5f;
    private static final float shoulderDownSpeed = 0.2f;
    
    private double stepTargetX, stepTargetY = 0;
    
    public AutoMode3Totes(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveToTest Constructor");
    }    
    
    private double shoulderSetpoint;
    private enum shoulderStates_e {
    	OFF,
    	UP,
    	DOWN
    }
    private shoulderStates_e shoulderState;
    
    public void autoInit() {
    	robot.locator.Reset();
    	lastTargetX = lastTargetY = -9999999;
    	currentStep = steps.STEP_SLIDE;
    	ToteCount = 0;
    	shoulderState = shoulderStates_e.OFF;
    	shoulderSetpoint = 0;
    	shoulderSetpoint = 0.9;
    	shoulderState = shoulderStates_e.UP;
    }
    


    public void autoPeriodic() {
    	//Shoulder logic 
    	switch(shoulderState) {
    		case UP:
    			if(robot.shoulder.getRawPosition() >= shoulderSetpoint) {
    				robot.shoulder.set(0);
    				shoulderState = shoulderStates_e.OFF;
    			} else {
    				robot.shoulder.set(shoulderUpSpeed);
    			}
    			break;
    		case DOWN:
    			if(robot.shoulder.getRawPosition() <= shoulderSetpoint) {
    				robot.shoulder.set(0);
    				shoulderState = shoulderStates_e.OFF;
    			} else {
    				robot.shoulder.set(-1.0*shoulderDownSpeed);
    			}
    			break;
    		case OFF:
    		default:
    			robot.shoulder.set(0);
    			break;
    	}
    	
    	//Main state machine
        switch(currentStep) {
        	
            case STEP_SLIDE:
            	robot.clawMotor1.set(Relay.Value.kReverse);
            	robot.clawMotor2.set(Relay.Value.kReverse);
            	
            	if(DriveToSlide(0, slideDistance, 1, 5)) {
            		currentStep = steps.STEP_DRIVE_CLOSE;;
            		robot.drive.tankDrive(0, 0);
            		robot.slideMotor1.set(0);
            		robot.slideMotor2.set(0);
            		robot.shoulder.set(0);
            		
            		shoulderSetpoint = upAngles[ToteCount];
                	shoulderState = shoulderStates_e.UP;
            	} 
            	break;
            case STEP_DRIVE_CLOSE:
            	if(DriveToSlide(toteDistances[ToteCount], robot.locator.GetY(), 0.75, 5))
            	{
            		robot.drive.tankDrive(0, 0);
            		currentStep = steps.STEP_SLIDE_IN;
            		steer_reset();
            	}
            	break;
            case STEP_SLIDE_IN:
            	/*if(DriveToSlide(robot.locator.GetX(), 0, 1, 2)) {
            		//currentStep = steps.STEP_DRIVE_TO_SENSOR;
            		currentStep = steps.STEP_DONE;
            		robot.drive.tankDrive(0, 0);
            		robot.slideMotor1.set(0);
            		robot.slideMotor2.set(0);
            	} */
            	if(robot.locator.GetY() <  -1) {
            		robot.slideMotor1.set(1);
            		robot.slideMotor2.set(-1);
            		steer();
            	} else {
            		robot.slideMotor1.set(0);
            		robot.slideMotor2.set(0);
            		robot.drive.tankDrive(0, 0);
            		currentStep = steps.STEP_PUT_THINGS_DOWN;
            	}
            	break;	
            case STEP_DRIVE_TO_SENSOR:
            	if(robot.rangeFinder.get() < canLowerDistance) {
            		robot.drive.tankDrive(0, 0);
            		currentStep = steps.STEP_PUT_THINGS_DOWN;
            	} else {
            		robot.drive.tankDrive(-0.5, -0.5);
            	}
            	break;
            case STEP_PUT_THINGS_DOWN:
            	//may have to lower gently...
            	robot.clawPneumatic.set(true);
            	shoulderSetpoint = downAngle;
            	shoulderState = shoulderStates_e.DOWN;
            	currentStep = steps.STEP_STEP_FWD;
            	stepTargetX = robot.locator.GetX() + 12;
       		   	stepTargetY = robot.locator.GetY();
            	break;
            case STEP_STEP_FWD:
            	if(DriveToSlide(stepTargetX, stepTargetY, 1, 5)) {
            		robot.slideMotor1.set(0);
            		robot.slideMotor2.set(0);
            		robot.drive.tankDrive(0, 0);
            		currentStep = steps.STEP_SUCK_IN;
            		lastEventTime = System.currentTimeMillis();
            	}
            	break;
            case STEP_SUCK_IN:
            	if(System.currentTimeMillis() - lastEventTime > suckInTime)
            	{
            		currentStep = steps.STEP_DONE;
            		/*
            		robot.clawMotor1.set(Relay.Value.kOff);
	            	robot.clawMotor2.set(Relay.Value.kOff);
	            	if(ToteCount < 1) {
	            		ToteCount++;
	            		currentStep = steps.STEP_SLIDE;
	            	} else {
	            		currentStep = steps.STEP_SLIDE_TO_AUTOZONE;
	            	}*/
            	}
            	else
            	{
	            	robot.clawPneumatic.set(false);
            	}
            	
            	break;
            case STEP_SLIDE_TO_AUTOZONE:
            	if(DriveToSlide(0, slideDistanceToAuto, 1, 5)) {
            		currentStep = steps.STEP_PUT_DOWN_IN_AUTOZONE;
            	} 
            	
            	if(robot.shoulder.getPosition() >= 10) {
         		   robot.shoulder.set(0);
            	} else {
            	   robot.shoulder.set(1);
            	}
            	
            	break;
            case STEP_PUT_DOWN_IN_AUTOZONE:
            	if(robot.shoulder.getPosition() <= downAngle) {
           		   robot.shoulder.set(0);
           		   lastEventTime = System.currentTimeMillis();
           		   currentStep = steps.STEP_BACK_AWAY_SLOWLY;
           		   robot.clawPneumatic.set(true);
              	} else {
              	   robot.shoulder.set(-1);
              	}
             	break;
            case STEP_BACK_AWAY_SLOWLY:
            case STEP_DONE:
            default:
                robot.drive.tankDrive(0.0,0.0);
                robot.slideMotor1.set(0);
                robot.slideMotor2.set(0);
                robot.clawMotor1.set(Relay.Value.kOff);
            	robot.clawMotor2.set(Relay.Value.kOff);
            	
        }
        SmartDashboard.putString("StepCount", currentStep.toString());
    }
       
    public String getName() {
        return "3 Totes";
    }
    
	@Override
	public String GetColumNames() {
		return "StepCount, ToteCount";
	}


	@Override
	public String log() {
		return String.format("%d,%d", currentStep, ToteCount);
	}
}
