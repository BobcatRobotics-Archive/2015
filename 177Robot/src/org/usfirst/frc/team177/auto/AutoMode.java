package org.usfirst.frc.team177.auto;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team177.robot.Robot;
import org.usfirst.frc.team177.trajectory.*;
import org.usfirst.frc.team177.robot.Constants;
import org.usfirst.frc.team177.trajectory.TrajectoryGenerator;
import org.usfirst.frc.team177.lib.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/* NOTE trajectory stuff currently doens't work! */

/**
 * Abstract class to provide the framework for handling multiple autonomous modes
 * 
 */

public abstract class AutoMode implements Logable {

	Robot robot; //reference to main implementation
	
    BasicPIDController DrivePID;
    BasicPIDController SteerPID;
    
    double lastRanDriveTo = 0;
    
    //TODO - convert these to use the Constants interface
    /* Variables & Constants used for DriveTo PID Controls */ 
    private static double SteerMargin = 5.0; //Margin to consider robot facing target (degrees)
    private static double DriveMargin = 10.0; //Margin to consider the robot at target (in)
    
    private static double DriveP = 0.25;  //Preportial gain for Drive System
    private static double DriveI = 0.0;   //Integral gain for Drive System
    private static double DriveD = 0.0;   //Derivative gain for Drive System
    private static double DriveMax = 1;   //Max Saturation value for control
    private static double DriveMin = -1;  //Min Saturation value for control
    
    private static double SteerP = 0.01; //0.02;  //Preportial gain for Steering System
    private static double SteerI = 0.01; //0.01 //Integral gain for Steering System
    private static double SteerD = 0.00;  //Derivative gain for Steering System
    private static double SteerMax = 1;   //Max Saturation value for control
    private static double SteerMin = -1;  //Min Saturation value for control
	
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
        
        DrivePID = new BasicPIDController(DriveP,DriveI,DriveD);
        DrivePID.setOutputRange(DriveMin, DriveMax);
        SteerPID = new BasicPIDController(SteerP,SteerI,SteerD);
        SteerPID.setOutputRange(SteerMin, SteerMax);
	}
	
	public abstract void autoPeriodic();
    public abstract String getName();
    
    private Timer driveTimer;
    
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    }
    
    public boolean DriveToSideways(double x, double y, double speed) 
    {
    	//Todo - need to implement this
    	return true;
    }
    
    /**
     * 
     * Drive the robot to the specified location
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
    public boolean DriveTo(double x, double y, double speed) 
    {
    	double steer, drive;
        //Reinitalize if the target has changed
        if(x != lastTargetX || y != lastTargetY) {
            lastTargetX = x;
            lastTargetY = y;
            DrivePID.reset();
            SteerPID.reset();
            lastRanDriveTo = System.currentTimeMillis();
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);            
        }
        //Calculate time step
        double now = System.currentTimeMillis();
        double dT = (now - lastRanDriveTo);
        lastRanDriveTo = now;
                
        double deltaX = x - robot.locator.GetX();
        double deltaY = y - robot.locator.GetY();
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        //System.out.println("DeltaX: "+deltaX+"  DeltaY: "+deltaY);
        System.out.println("Distance: " + distance);
        //determine angle to target relative to field
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));  // +/- 180 degrees
        System.out.println("Target Heading: "+targetHeading);
        if(speed < 0) {
            //reverse heading if going backwards
            targetHeading += 180;
        }
        
        //Determine  angle to target relative to robot
        
        double bearing = (targetHeading + robot.locator.GetHeading())%360;
        if (bearing > 180) {
            bearing = bearing - 360; //Quicker to turn the other direction
        }
        //System.out.println("bearing: "+bearing);
        /* Steering PID Control */
        steer = SteerPID.calculate(bearing, dT);
        //System.out.println("BEARING: "+bearing);
        
        /* Drive PID Control */                
        if(speed == 0) {
            //Just turn to the target, no PI Control
            drive = 0;
        } else {
            drive = -1.0*DrivePID.calculate(distance, dT)*speed;
        }        

        //Move the robot - Would this work better if we multiplyed by the steering PID output?
        //System.out.println("DRIVE: "+drive);
        //System.out.println("STEER: "+steer);
        robot.drive.tankDrive(drive+steer, drive-steer);

                
        if((distance < DriveMargin) || (Math.abs(bearing) < SteerMargin && speed == 0 )) {
            return true;
        } else {
            return false;
        }        
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
    	    
            Trajectory reference = TrajectoryGenerator.generate(config, strategy, robot.locator.GetVel(), robot.locator.GetHeading(), distance, 0, goal_heading);
    
            followerLeft.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
            followerRight.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
            
            Trajectory leftProfile = reference;
            Trajectory rightProfile = reference.copy(); // Copy

            //adjust trajectory for turn
            /* Disabled for now
            double deltaHeading = goal_heading - robot.locator.GetHeading();
            if(deltaHeading != 0)
            {
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
            } */
            
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
    
    /*Made protected so that they can be logged in the specific automode */
    protected double distanceL;
    protected double distanceR;
    protected double speedLeft;
    protected double speedRight;
    protected double goalHeading;
    protected double observedHeading;
    protected double angleDiff;
    protected double turn;
    
    //Update ongoing 
    public void Update() {
    	double direction = -1; //set to -1 to go backwards?
    	
    	if (followerLeft.isFinishedTrajectory()) {
    		robot.drive.tankDrive(0.0, 0.0);
    		stopDriveTimer();
   	    } else  {
   	    	distanceL = direction * robot.locator.getLeftEncoderDistance();
    	    distanceR = direction * robot.locator.getRightEncoderDistance();

    	    speedLeft = direction * followerLeft.calculate(distanceL);
    	    speedRight = direction * followerRight.calculate(distanceR);
    	      
    	    goalHeading = followerLeft.getHeading();
    	    observedHeading = robot.locator.getGyroAngleInRadians();

    	    double angleDiffRads = BobcatUtils.getDifferenceInAngleRadians(observedHeading, goalHeading);
    	    angleDiff = Math.toDegrees(angleDiffRads);

    	    turn = Constants.kTurn.getDouble() * angleDiff;
    	    robot.drive.tankDrive(speedLeft + turn, speedRight - turn);
   	    }
    }
    
    //Default logging behavior is to log nothing
	@Override
	public String GetColumNames() {
		return null;
	}


	@Override
	public String log() {
		return null;
	}
}
