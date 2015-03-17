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
public class AutoModeToteTunr90java extends AutoMode {
    
    private int StepCount = 0;
    private long lastMs =0;

    public AutoModeToteTunr90java(Robot robot) {
        super(robot);
        System.out.println("AutoModeToteTunr90java Constructor");
    }
    
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    	StepCount = 0;
    	robot.locator.Reset();
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
            	//robot.lifter.set(true);
            	lastMs = System.currentTimeMillis();
            	StepCount++;
            	break;
            case 1:
            	if(System.currentTimeMillis() - lastMs > 200) {
            		StepCount++;
            	}
            	break;
            case 2:
            	robot.drive.tankDrive(-0.5,0.5);
            	if(robot.locator.GetHeading() > 80 && robot.locator.GetHeading() < 92) {
            		robot.drive.tankDrive(0.0,0.0);
            		StepCount++;
            	}
            	break;
            case 3:
            	robot.drive.tankDrive(-0.8,-0.8);
            	if(robot.locator.GetY() < -180) {
            		robot.drive.tankDrive(0.0,0.0);
            		StepCount++;
            	}
            	break;
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
	SmartDashboard.putNumber("StepCount", StepCount);
    }
       
    public String getName() {
        return "Tote Turn 90";
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
