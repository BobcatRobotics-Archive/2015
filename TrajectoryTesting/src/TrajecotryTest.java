import org.usfirst.frc.team177.lib.BobcatUtils;
import org.usfirst.frc.team177.robot.Constants;
import org.usfirst.frc.team177.trajectory.Trajectory;
import org.usfirst.frc.team177.trajectory.TrajectoryFollower;
import org.usfirst.frc.team177.trajectory.TrajectoryGenerator;


public class TrajecotryTest {
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		double x = 12;
		double y = 0;
		double goal_heading = 0; 

		TrajectoryFollower followerLeft = new TrajectoryFollower("left");
		TrajectoryFollower followerRight = new TrajectoryFollower("right");
		
		TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
	    config.dt = Constants.updateDt.getDouble()/1000.0;
	    config.max_acc = Constants.maxAcc.getDouble();
	    config.max_jerk = Constants.maxJerk.getDouble();
	    config.max_vel = Constants.maxVel.getDouble();
	    
	    TrajectoryGenerator.Strategy strategy = TrajectoryGenerator.SCurvesStrategy;
	    
	    double deltaX = x - 0;// robot.locator.GetX();
        double deltaY = y - 0;//robot.locator.GetY();
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        
        double velocity = 0; //robot.locator.GetVel()
        double headingInRads = 0; //robot.locator.GetHeadingRadians()
        
        Trajectory reference = TrajectoryGenerator.generate(config, strategy, velocity, headingInRads, distance, 0, goal_heading);

        followerLeft.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
        followerRight.configure(Constants.trajKp.getDouble(), Constants.trajKi.getDouble(), Constants.trajKd.getDouble(), Constants.trajKv.getDouble(), Constants.trajKa.getDouble()); 
        
        Trajectory leftProfile = reference;
        Trajectory rightProfile = reference.copy(); // Copy

        //adjust trajectory for turn
        double deltaHeading = goal_heading -  headingInRads;
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
        }
        
        followerLeft.setTrajectory(leftProfile);
        followerRight.setTrajectory(rightProfile);
        
        double direction = 1;
        int loop = 0;
    	while(!followerLeft.isFinishedTrajectory())
    	{
    		double distanceL = direction * 0; //robot.locator.getLeftEncoderDistance();
    	    double distanceR = direction * 0; //robot.locator.getRightEncoderDistance();

    	    double speedLeft = direction * followerLeft.calculate(distanceL);
    	    double speedRight = direction * followerRight.calculate(distanceR);
    	      
    	    double goalHeading = followerLeft.getHeading();
    	    double observedHeading = 0; //robot.locator.getGyroAngleInRadians();

    	    double angleDiffRads = BobcatUtils.getDifferenceInAngleRadians(observedHeading, goalHeading);
    	    double angleDiff = Math.toDegrees(angleDiffRads);

    	    double turn = Constants.kTurn.getDouble() * angleDiff;
    	    //robot.drive.tankDrive(speedLeft + turn, speedRight - turn);
    	    System.out.printf("%d: left: %f right: %f%n", loop++, speedLeft + turn, speedRight - turn);
    	}
	}

	
}
