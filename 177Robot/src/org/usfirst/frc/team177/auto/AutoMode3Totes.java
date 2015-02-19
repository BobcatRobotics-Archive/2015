/*
 * Basic Automode to test frame work, drive forward for 3 feet then turn 90 degrees to right
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author schroed
 */
public class AutoMode3Totes extends AutoMode {
    
    private int StepCount = 0;
    private long lastEventTime = 0;;
    private int ToteCount = 0;
    
    private static final double slowSpeed = 0.5;
    private static final double fullSpeed = 1.0;
    
    /* Timing parameters */
    private static final long liftDownDelay = 200;
    private static final long liftUpDelay = 200;
    private static final long stackOpenDelay = 200;
    
    /* Distances between totes */
    private static final long toteOffsetX = 72;
    private static final long toteOffsetY = 48;

    public AutoMode3Totes(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveToTest Constructor");
    }    
    
    public void autoInit() {
    	StepCount = 0;
    	ToteCount = 0;
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
            	//Turn on one pickup motor and drive forward until we hit the tote or have gone 1 foot
            	robot.pickupMotor1.set(fullSpeed);
            	if(DriveTo(12+(ToteCount+toteOffsetX),ToteCount*toteOffsetY,slowSpeed) || robot.toteSensor.get()) {
            		robot.drive.tankDrive(0.0,0.0);
                    StepCount++;
                    lastEventTime = System.currentTimeMillis();
                } else {
                	//Check to see if we're close enough to close the arms?
                	robot.lowArmsPickup.set(false);
                }
                break;
            case 2:
            	//Lower lifter
            	robot.lifter.set(true);            	
            	if(System.currentTimeMillis() - lastEventTime > liftDownDelay)
            	{
            		StepCount++;
            		lastEventTime = System.currentTimeMillis();
            	}
            	break;
            case 3:
            	//raise lifter
            	robot.lifter.set(false);            	
            	if(System.currentTimeMillis() - lastEventTime > liftUpDelay)
            	{            		
            		lastEventTime = System.currentTimeMillis();
            		if(ToteCount < 2)
            		{
            			ToteCount++;
            			StepCount++;
            		}
            		else
            		{
            			//picked up the third tote
            			StepCount = 5;
            		}

            	}
            	break;
            case 4:
            	//Open pickup arms and drive sideways tangent with next tote
            	robot.lowArmsPickup.set(true);
            	if(DriveToSideways(12+(ToteCount+toteOffsetX), ToteCount*toteOffsetY ,slowSpeed)) {
            		robot.drive.tankDrive(0.0,0.0);
            		StepCount = 0; //go back to the start of the cycle and pick up again
                    lastEventTime = System.currentTimeMillis();
                }            	
                break;
            case 5:
            	//Back into autonomous zone 
            	if(DriveTo(0,2*toteOffsetY,-1.0*slowSpeed) || robot.toteSensor.get()) {
            		robot.drive.tankDrive(0.0,0.0);
                    StepCount++;
                    robot.lowArmsPickup.set(true); //expand the pickup arms
                    lastEventTime = System.currentTimeMillis();
                }
            case 6:
            	//Lower the lifter
            	robot.lifter.set(true);            	
            	if(System.currentTimeMillis() - lastEventTime > liftDownDelay)
            	{
            		StepCount++;
            		lastEventTime = System.currentTimeMillis();
            	}
            	break;
            case 7:
            	//open the stacker
            	robot.highBoxPickup.set(true);            	
            	if(System.currentTimeMillis() - lastEventTime > stackOpenDelay)
            	{
            		StepCount++;
            		lastEventTime = System.currentTimeMillis();
            	}
            	break;
            case 8:
            	//back away slowly
            	if(DriveTo(-24.0,2*toteOffsetY,-1.0*slowSpeed) || robot.toteSensor.get()) {
            		robot.drive.tankDrive(0.0,0.0);
                    StepCount++;
                    lastEventTime = System.currentTimeMillis();
                }
            	break;
            case 9:
            	//Cleanup
            	robot.highBoxPickup.set(false);
            	robot.lifter.set(false);          
            	robot.lowArmsPickup.set(false);
            	StepCount++;
            	break;
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
        SmartDashboard.putNumber("StepCount", StepCount);
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
		return String.format("%d,%d", StepCount, ToteCount);
	}
}
