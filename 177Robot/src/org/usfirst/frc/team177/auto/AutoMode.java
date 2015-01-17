package org.usfirst.frc.team177.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team177.robot.Robot;
import org.usfirst.frc.team177.trajectory.*;
import org.usfirst.frc.team177.robot.Constants;
import org.usfirst.frc.team177.trajectory.TrajectoryGenerator;
import org.usfirst.frc.team177.lib.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Abstract class to provide the framework for handling multiple autonomous modes
 * 
 */

public abstract class AutoMode {

	Robot robot; //reference to main implementation
	
	double startX;
	double startY;
	
	double lastTargetX = 0;
    double lastTargetY = 0;
    Trajectory curTrajectory;
	
    double lastRunDriveTo;
    
    TrajectoryFollower followerLeft = new TrajectoryFollower("left");
    TrajectoryFollower followerRight = new TrajectoryFollower("right");
    
	//Constructor
	public AutoMode(Robot robot) {
        this.robot = robot;
	}
	
	public abstract void autoPeriodic();
    public abstract String getName();
    
    private Timer driveTimer;
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    }
    
    /**
     * 
     * Drive the robot to the specified location using a trajectory
     * 
     *      -----------    +
     *      | Robot   |
     *      |   --->  |    Y
     *      |         |   
     *      -----------    -
     *       -   X    +
     *  Robot starts match at 0,0 heading 0
     * 
     * @param x - x coordinate of target
     * @param y - y coordinate of target
     * @param speed - Speed to drive, a negative value will cause the robot to backup.
     *                A Speed of 0 will cause the robot to turn to the target without moving
     * @return - Boolean value indicating if the robot is at the target or not (true = at target).
     * @author schroed
     */     
    public boolean DriveToToTrajectory(double x, double y, double goal_heading) 
    {
    	//Reinitalize if the target has changed
        if(x != lastTargetX || y != lastTargetY) {
        	//Build a trajectory
            lastTargetX = x;
            lastTargetY = y;
            startX = robot.locator.GetX();
            startY = robot.locator.GetY();
            
            TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
    	    config.dt = Constants.updateDt.getDouble()/1000.0;
    	    config.max_acc = Constants.maxAcc.getDouble();
    	    config.max_jerk = Constants.maxJerk.getDouble();
    	    config.max_vel = Constants.maxVel.getDouble();
    	    
    	    TrajectoryGenerator.Strategy strategy = TrajectoryGenerator.SCurvesStrategy;
    	    
    	    double deltaX = x - robot.locator.GetX();
            double deltaY = y - robot.locator.GetY();
            double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
    	    
            Trajectory reference = TrajectoryGenerator.generate(config, strategy, robot.locator.GetVel(), robot.locator.GetHeadingRadians(), distance, 0, goal_heading);
    
            followerLeft.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
            followerRight.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
            
            Trajectory leftProfile = reference;
            Trajectory rightProfile = reference.copy(); // Copy

            //adjust trajectory for turn
            double deltaHeading = goal_heading - robot.locator.GetHeading();
            double radius = Math.abs(Math.abs(distance) / (deltaHeading * Math.PI / 180.0));
            double width = Constants.robotWidth.getDouble();
            double faster = (radius + (width / 2.0)) / radius;
            double slower = (radius - (width / 2.0)) / radius;
            System.out.println("faster " + faster);

            if (deltaHeading > 0) {
              leftProfile.scale(faster);
              rightProfile.scale(slower);
            } else {
              leftProfile.scale(slower);
              rightProfile.scale(faster);
            }
            
            followerLeft.setTrajectory(leftProfile);
            followerRight.setTrajectory(rightProfile);
            
            //DrivePID.reset();
            //SteerPID.reset();
            //lastRanDriveTo = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);
            
            startDriveTimer();
        }
        return followerLeft.isFinishedTrajectory();
    }

    private class AutoDriveTask extends TimerTask {
        
    	private AutoMode am;
    	
    	AutoDriveTask(AutoMode am)
    	{
    		this.am = am;    	
    	}

		@Override
		public void run() {
			am.Update();			
		}
    	
    }

    
    private void startDriveTimer() {
    	if(driveTimer == null)
    	{
    		driveTimer = new Timer();
    		AutoDriveTask adt = new AutoDriveTask(this);
    		driveTimer.schedule(adt, 0L, (long)Constants.updateDt.getInt());
    	}
    }
    
    private void stopDriveTimer() {
    	if(driveTimer != null)
    	{
    		driveTimer.cancel();
    	    driveTimer = null;
    	}
    }
    
    
    //Update ongoaing 
    public void Update() {
    	double direction = -1; //set to -1 to go backwards?
    	
    	if (followerLeft.isFinishedTrajectory()) {
    		robot.drive.tankDrive(0.0, 0.0);
    		stopDriveTimer();
   	    } else  {
   	    	double distanceL = direction * robot.locator.getLeftEncoderDistance();
    	    double distanceR = direction * robot.locator.getRightEncoderDistance();

    	    double speedLeft = direction * followerLeft.calculate(distanceL);
    	    double speedRight = direction * followerRight.calculate(distanceR);
    	      
    	    double goalHeading = followerLeft.getHeading();
    	    double observedHeading = robot.locator.getGyroAngleInRadians();

    	    double angleDiffRads = BobcatUtils.getDifferenceInAngleRadians(observedHeading, goalHeading);
    	    double angleDiff = Math.toDegrees(angleDiffRads);

    	    double turn = Constants.kTurn.getDouble() * angleDiff;
    	    robot.drive.tankDrive(speedLeft + turn, speedRight - turn);
   	    }
    }
}
