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
public class AutoModeCanSlide extends AutoMode {
    
    private int StepCount = 0;
    private long LastMS;

    public AutoModeCanSlide(Robot robot) {
        super(robot);
        System.out.println("AutoModeCanSlide Constructor");
    }
    
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    	StepCount = 0;
    	robot.locator.Reset();
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
            	robot.shoulder.set(1);
            	if(robot.shoulder.getRawPosition() > 1.2) {
            		robot.shoulder.set(0);
            		StepCount++;
            	}
            	break;
            case 1:
            	robot.slideMotor1.set(1);
            	robot.slideMotor2.set(-1);
            	if(robot.locator.GetY() < -112) {
            		robot.slideMotor1.set(0);
                	robot.slideMotor2.set(0);
            	}
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
	SmartDashboard.putNumber("StepCount", StepCount);
    }
       
    public String getName() {
        return "CanSlide";
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
