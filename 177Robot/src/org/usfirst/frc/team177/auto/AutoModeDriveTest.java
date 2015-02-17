package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.Robot;

public class AutoModeDriveTest extends AutoMode {
	int StepCount = 0;
    int SubStepCount = 0;
    
	
	public AutoModeDriveTest(Robot robot) {
		super(robot);
	}

	@Override
    public void autoPeriodic() {
        System.out.println("Step Cnt"+StepCount);
        switch(StepCount) {
           case 0:
               //Drive forward for approx 1/2 second
               robot.drive.tankDrive(-0.5,-0.5);
               if(SubStepCount > 50) {
                   StepCount++;
                   SubStepCount = 0;
               }
               SubStepCount++;
               break;
           case 1:
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
       return "Basic Drive Test";
   }


}
