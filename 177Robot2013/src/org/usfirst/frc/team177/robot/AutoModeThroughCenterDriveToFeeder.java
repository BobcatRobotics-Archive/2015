/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.robot;

/**
 *
 * @author Robotics
 */
public class AutoModeThroughCenterDriveToFeeder extends AutoMode
{

    private int stepCount = 0;

    public AutoModeThroughCenterDriveToFeeder(Team177Robot robot)
    {
	super(robot);
    }

    public void autoPeriodic()
    {
        System.out.println("stepcount:" +stepCount);
	switch(stepCount)
	{
	    case 0: 
		//Shoot 3 times
                robot.peg.set(true);
		robot.shooter.Fire(6);                
                stepCount++;
                //robot.shifter.set(true); //High Gear!
		break;
            case 1:
                if(robot.shooter.isDone()) {
                    stepCount++;
                    robot.peg.set(false);
                }
                break;
            case 2:
                //Driver forward toward center line
                if(DriveTo(50,0,1.0)) { //58
                    stepCount++;
                }
                break; 
            case 3:
            /*    //Turn to left
                if(DriveTo(100,100,0)) {//138
                    stepCount++;
                }
                break;
            case 4:*/
                //Driver forward toword edge of field
                if(DriveTo(120,100,1.0)) {
                    stepCount++;
                }
                break;                
           /* case 5:
                //Turn to face feeder station
                if(DriveTo(380,148,0)) {
                    stepCount++;
                }
                break;*/                                
            default:
		robot.drive.tankDrive(0.0,0.0);
	}
    }

    public String getName()
    {
	return "Shoot, Drive To Feeder";
    }
}
