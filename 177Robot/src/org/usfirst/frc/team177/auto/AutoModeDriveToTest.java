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
public class AutoModeDriveToTest extends AutoMode {
    
    private int StepCount = 0;

    public AutoModeDriveToTest(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveToTest Constructor");
    }
    
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    	StepCount = 0;
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
                //Drive Forward 3 feet
                if(DriveToToTrajectory(36,0,0)) {
                    StepCount++;
                }
                break;
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
	SmartDashboard.putNumber("StepCount", StepCount);
    }
       
    public String getName() {
        return "DriveToTest";
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
