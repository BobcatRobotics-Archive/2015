/*
 * Basic Automode to test frame work, drive forward for 3 feet then turn 90 degrees to right
 */
package org.usfirst.frc.team177.robot;


/**
 *
 * @author schroed
 */
public class AutoModeParkTest extends AutoMode {
    
    private int StepCount = 0;

    public AutoModeParkTest(Team177Robot robot) {
        super(robot);
        System.out.println("AutoModeParkTest Constructor");
    }

    public void autoPeriodic() {
        switch(StepCount) {
            case 0:
                //Drive Forward 3 feet, left 1
                if(Park(36,-1,0,1.0)) {
                    StepCount++;
                }
                break;
            default:
                robot.drive.tankDrive(0.0,0.0);
        }
    }
       
    public String getName() {
        return "Park Test";
    }
}
