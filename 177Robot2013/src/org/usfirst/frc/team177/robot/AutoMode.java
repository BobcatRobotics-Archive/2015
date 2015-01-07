/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract class to provide the framework for handling multiple autonomous modes
 * @author schroed
 */
public abstract class AutoMode {
    
    Team177Robot robot; //reference to main implementation
    
    BasicPIDController DrivePID;
    BasicPIDController SteerPID;
    
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
    
    double lastTargetX = 0;
    double lastTargetY = 0;
    double planeSlope = 0;
    double planeIntercept = 0;
    
    
    double lastRanDriveTo = 0;

    public AutoMode(Team177Robot robot) {
        this.robot = robot;
        
        DrivePID = new BasicPIDController(DriveP,DriveI,DriveD);
        DrivePID.setOutputRange(DriveMin, DriveMax);
        SteerPID = new BasicPIDController(SteerP,SteerI,SteerD);
        SteerPID.setOutputRange(SteerMin, SteerMax);
    }
        
    public abstract void autoPeriodic();
    public abstract String getName();
    
    public void autoInit() {
        DrivePID.reset();
        SteerPID.reset();
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
            lastRanDriveTo = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);            
        }
        //Calculate time step
        double now = Timer.getFPGATimestamp();
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
     * Drive the robot to the specified location, will stop when it crosses the plane tangent to the initial heading, through the target point
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
    public boolean DriveToPlane(double x, double y, double speed) 
    {
        double steer, drive;
        
        double deltaX = x - robot.locator.GetX();
        double deltaY = y - robot.locator.GetY();
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        //System.out.println("DeltaX: "+deltaX+"  DeltaY: "+deltaY);
        //determine angle to target relative to field
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));  // +/- 180 degrees
        //System.out.println("Target Heading: "+targetHeading);
        if(speed < 0) {
            //reverse heading if going backwards
            targetHeading += 180;
        }
                
        //Reinitalize if the target has changed
        if(x != lastTargetX || y != lastTargetY) {
            lastTargetX = x;
            lastTargetY = y;
            DrivePID.reset();
            SteerPID.reset();
            
            if(deltaX == 0) {
                planeSlope = Float.POSITIVE_INFINITY;
            } else {
                planeSlope = -1.0*(deltaY/deltaX);
                planeIntercept = y-(planeSlope*x);
            }            
            
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);  
            lastRanDriveTo = Timer.getFPGATimestamp();
        }
        //Calculate time step
        double now = Timer.getFPGATimestamp();
        double dT = (now - lastRanDriveTo);
        lastRanDriveTo = now;
                        
        //Determine  angle to target relative to robot
        
        double bearing = (targetHeading + robot.locator.GetHeading())%360;
        if (bearing > 180) {
            bearing = bearing - 360; //Quicker to turn the other direction
        }
        
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
                
        if(planeSlope == 0.0f ) {
            //Driving along the X axis, ignore Y
            return Math.abs(deltaX) < DriveMargin;
        } else if (planeSlope == Float.POSITIVE_INFINITY) {
            //Driving along the Y axis, ignore X
            return Math.abs(deltaY) < DriveMargin;
        } else if (planeIntercept >= 0.0f ) {            
            return robot.locator.GetY() >= robot.locator.GetX()*planeSlope+planeIntercept;
        } else {
            return robot.locator.GetY() <= robot.locator.GetX()*planeSlope+planeIntercept;
        } 
    }
    
    

    /**
     *
     * Drive the robot to the specified location using the park algorithm
     *
     *      -----------    +
     *      | Robot   |
     *      |   --->  |    Y
     *      |         |
     *      -----------    -
     *       -   X    +
     *  Robot starts match at 0,0 heading 0
     *
     *  Based on: http://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
     *
     * @param x - x coordinate of target
     * @param y - y coordinate of target
     * @param theta - desired parking angle
     * @param speed - Speed to drive, a negative value will cause the robot to backup.
     *                A Speed of 0 will cause the robot to turn to the target without moving
     * @return - Boolean value indicating if the robot is at the target or not (true = at target).
     * @author schroed
     */
    public boolean Park(double x, double y, double theta, double speed)  {
        //Control constants
        final double k1 = 2.0d;  //This controls the drive speed
        final double k2 = 5.0d;  //This controls the Steering athority
        final double k3 = 2.0d;
        final double width = 28; //width of the robot in inches
        final double maxSpeed = 132; // in/s for robot at fulll power

        double v, w, steer, drive, targetHeading;
        //Reinitalize if the target has changed
        if(x != lastTargetX || y != lastTargetY) {
            lastTargetX = x;
            lastTargetY = y;
            lastRanDriveTo = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);
        }

        double deltaX = robot.locator.GetX() - x;
        double deltaY = robot.locator.GetY() - y;
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        System.out.println(deltaX + " " + deltaY + " " + distance);

        //determine angle to target relative to field
        if(deltaX == 0d && deltaY == 0d) {
            targetHeading = 0d;
        } else {
            targetHeading = (Math.atan2(deltaX, deltaY) - Math.toRadians(robot.locator.GetHeading())) +Math.PI;  // radians
        }
        if(targetHeading > Math.PI) {
            targetHeading -= 2*Math.PI;
        }
        System.out.println("targetHeading: " + targetHeading);
        if(speed < 0) {
            //reverse heading if going backwards
            targetHeading -= Math.PI;
        }

        //calculate drive and steering in f/s and rad/s
        v = k1*distance*Math.cos(targetHeading);
        if(targetHeading == 0d) {
            w = 0d;
        } else {
            w = (k2*targetHeading) + k1*((Math.sin(targetHeading)*Math.cos(targetHeading))/targetHeading)*(targetHeading+k3*(targetHeading+Math.toRadians(theta)));
        }

        //Limit Control Outputs
        double Vmax = maxSpeed*speed; // f/s - TODO This assumes linear speed response... Validate this
        double Wmax = (Vmax*2)/width; // rad/s - TODO Validate this
        double V_Vmax = Math.abs(v)/Vmax;
        double W_Wmax = Math.abs(w)/Wmax;
        
        System.out.println(Vmax + " " + Wmax + " " + V_Vmax + " " + W_Wmax);
        if(V_Vmax < 1 && W_Wmax < 1) {
            //Simple case, outputs not limited
            drive = v;
            steer = w;
        } else if (V_Vmax > W_Wmax) {
            //limit based on driving velocity (v)
            if (v < 0) {
                drive = Vmax * -1;
            } else {
                drive = Vmax;
            }
            steer = w/V_Vmax;
        } else {
            //limit based on steering velcity (w)
            if(w < 0) {
                steer = Wmax * -1;
            } else {
                steer = Wmax;
            }
            drive = v/W_Wmax;
        }

        steer *= (width/2);
        System.out.println("Steer: "+steer+" Drive: "+drive);
        //Move the robot - scale outputs to 0-1 range
        robot.drive.tankDrive(-1.0*((drive+steer)/maxSpeed), -1.0*((drive-steer)/maxSpeed));
        System.out.println("tankDrive: " + (-1.0*((drive+steer)/maxSpeed)) +" "+ (-1.0*((drive-steer)/maxSpeed)));

        if((distance < DriveMargin) || (Math.abs(Math.toDegrees(targetHeading)) < SteerMargin && speed == 0 )) {
            return true;
        } else {
            return false;
        }   

    }

}
