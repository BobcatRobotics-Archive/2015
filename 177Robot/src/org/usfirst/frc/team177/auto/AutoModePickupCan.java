package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.Robot;

public class AutoModePickupCan extends AutoMode {
	int StepCount = 0;
    int SubStepCount = 0;
    
	
	public AutoModePickupCan(Robot robot) {
		super(robot);
	}

	@Override
    public void autoPeriodic() {
        System.out.println("Step Cnt"+StepCount);
        switch(StepCount) {
           case 0:
        	   //Close claw
        	   StepCount++;
        	   break;
           case 1: 
        	   //Raise shoulder to 25%
        	   //This requires the automatic position control to work
        	   //May need a delay
        	  // robot.shoulder.setTargetPosition(0.25);
        	   StepCount++;
        	   break;         	  
           case 2:
               //Drive forward for approx 1/2 second
        	   //TODO - make this use the Drive to logic
               robot.drive.tankDrive(-0.5,-0.5);
               if(SubStepCount > 50) {
                   StepCount++;
                   SubStepCount = 0;
               }
               SubStepCount++;
               break;
           case 3:
               StepCount++;
               robot.drive.tankDrive(0,0);
               break;
           default:
               robot.drive.tankDrive(0.0,0.0);
       }
   }

   @Override
   public void autoInit() {
       StepCount = 0;
       SubStepCount = 0;
   }
   
   @Override
   public String getName() {
       return "Pickup Can Drive Forward";
   }


}
