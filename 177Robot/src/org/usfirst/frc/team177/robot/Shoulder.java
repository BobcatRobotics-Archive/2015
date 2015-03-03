package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 */
public class Shoulder {

	Talon motor;
	AnalogInput pot;

    // Initialize your subsystem here
    public Shoulder(int MotorChannel, int PotChannel) {
    	motor = new Talon(MotorChannel);
    	pot = new AnalogInput(PotChannel);
    }
    
    /** Return position as a % of full travel **/
    public double getPosition() 
    {
    	return (pot.getVoltage() - Constants.shoulderMinV.getDouble())/(Constants.shoulderMaxV.getDouble()-Constants.shoulderMinV.getDouble());
    }
    
    /** Return the raw voltage from the pot for calibration **/
    public double getRawPosition() 
    {
    	return pot.getVoltage();
    }

    
    public void set(double setpoint)
    {
      	//Dead band
    	if(Math.abs(setpoint) > Constants.shoulderDeadband.getDouble())
    	{
    		//Limit logic
    		if((Constants.shoulderEnableLimits.getDouble()<1)
    	        || (setpoint > 0 && getRawPosition() < Constants.shoulderLimitUp.getDouble())
    			|| (setpoint < 0 && getRawPosition() > Constants.shoulderLimitDown.getDouble()))
    		{		
    			motor.set(setpoint);	
    		}
    		else
    		{
    			motor.set(0);
    		}
    	}
    	else
    	{
    		motor.set(0);    	
    	}    
    }
    
}
