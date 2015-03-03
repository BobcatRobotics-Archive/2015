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
public class AutoModeDriveForward extends AutoMode {
    
    private int StepCount = 0;

    public AutoModeDriveForward(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
    }
    
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    	StepCount = 0;
    	robot.locator.Reset();
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
            /*	robot.shoulder.set(1);
            	if(robot.shoulder.getRawPosition() > 1.2) {
            		robot.shoulder.set(0);
            		StepCount++;
            	}
            	break;
            case 1:*/
            	robot.drive.tankDrive(-0.8,-0.8);
            	if(robot.locator.GetY() < -160) {
            		robot.drive.tankDrive(0.0,0.0);
            		StepCount++;
            	}
            	/*if(DriveTo(150, 0, 1)) {
            		robot.drive.tankDrive(0.0,0.0);
            		StepCount++;
            	}*/
            	break;
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
	SmartDashboard.putNumber("StepCount", StepCount);
    }
       
    public String getName() {
        return "DriveForward";
    }

	@Override
	public String GetColumNames() {
		return "distanceL, distanceR, speedLeft, speedRight, goalHeading, observedHeading, angleDiff, turn" + "," + followerLeft.GetColumNames();
	}


	@Override
	public String log() {
		return String.format("%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f", distanceL, distanceR, speedLeft, speedRight, goalHeading, observedHeading, angleDiff, turn) + "," + followerLeft.log();
	}
}
