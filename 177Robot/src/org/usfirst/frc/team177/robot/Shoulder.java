package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Shoulder extends PIDSubsystem {

	Victor motor;
	AnalogInput pot;

    // Initialize your subsystem here
    public Shoulder(int MotorChannel, int PotChannel) {
    	super(Constants.shoulderKp.getDouble(), Constants.shoulderKi.getDouble(), Constants.shoulderKd.getDouble());
    	motor = new Victor(MotorChannel);
    	pot = new AnalogInput(PotChannel);
    	setSetpoint(getPosition());
    	if(Constants.shoulderEnableControl.getDouble() > 0)
    	{
    		//Control is enabled, start the PID Controller
    		enable();
    	}
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return getPosition();
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	set(output, true);
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
    	//Helper function to set motor from teleop code
    	set(setpoint, false);
    }
    
    public void set(double setpoint, boolean PIDInput)
    {
    	//Dead band
    	if(Math.abs(setpoint) < Constants.shoulderDeadband.getDouble())
    	{
    		//Limit logic
    		if((Constants.shoulderEnableLimits.getDouble()<1)
    	        || (setpoint > 0 && getPosition() < Constants.shoulderLimitUp.getDouble())
    			|| (setpoint < 0 && getPosition() > Constants.shoulderLimitDown.getDouble()))
    		{
    			if(!PIDInput && Constants.shoulderEnableControl.getDouble() > 0)
    	    	{
    	    		//Control is enabled, stop the control if this is manual input
    	    		disable();
    	    	}
    			motor.set(setpoint);	
    		}
    		else
    		{
    			motor.set(0);
    			if(!PIDInput) {
    				//Update our target position if the operator moved the arm
    				setSetpoint(getPosition());
    				if(Constants.shoulderEnableControl.getDouble() > 0)
    		    	{
    		    		//Control is enabled, start the PID Controller
    		    		enable();
    		    	}
    			}
    		}
    	}
    	else
    	{
    		motor.set(0);    	
    	}    
    }
    
    /** directly set the target control position, this should be useful for auto **/
    public void setTargetPosition(double target)
    {    	
    	setSetpoint(target);
    }
}
