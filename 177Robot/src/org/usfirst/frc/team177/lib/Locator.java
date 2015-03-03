/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team177.robot.*;

/**
 * Keep track of robots location in x,y grid with 0,0 being starting location
 */
public class Locator implements Logable {
    
    private final EnhancedGyro headingGyro;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder slide1Encoder;
    private final Encoder slide2Encoder;
    public final UpdateLocation updateLocation;
    
    public Locator() {
        headingGyro = new EnhancedGyro();
        leftEncoder = new Encoder(Constants.DIOLeftEncoderA.getInt(), Constants.DIOLeftEncoderB.getInt());
        rightEncoder = new Encoder(Constants.DIORightEncoderA.getInt(), Constants.DIORightEncoderB.getInt());
        slide1Encoder = new Encoder(Constants.DIOSlide1EncoderA.getInt(), Constants.DIOSlide1EncoderB.getInt());
        slide2Encoder = new Encoder(Constants.DIOSlide2EncoderA.getInt(), Constants.DIOSlide2EncoderB.getInt());
                
        //Set Default Values
        leftEncoder.setDistancePerPulse(Constants.LeftEncoderDPP.getDouble());
        rightEncoder.setDistancePerPulse(Constants.RightEncoderDPP.getDouble());
        slide1Encoder.setDistancePerPulse(Constants.SlideEncoderDPP.getDouble());
        slide2Encoder.setDistancePerPulse(Constants.SlideEncoderDPP.getDouble());
    
        updateLocation = new UpdateLocation();       
            
        LiveWindow.addSensor("Locater", "left Encoder", leftEncoder);
        LiveWindow.addSensor("Locater", "right Encoder", rightEncoder);
        LiveWindow.addSensor("Locater", "Gyro", headingGyro);
    }
    
    public void start() {
        updateLocation.start();
    }
    
    
    public double GetHeading() {
        return updateLocation.heading;
    }
    
    public double GetX() {
        return updateLocation.x;
    }
    
    public double GetY() {
        return updateLocation.y;
    }
    
    /* Set x/y location to 0,0 and heading to 0 degrees */
    public void Reset() {
        updateLocation.Reset();   
    }
    
    public double getLeftRaw() {
        return leftEncoder.getDistance();
    }
    
    public double getRightRaw() {
        return rightEncoder.getDistance();
    }
    
    public double getSlide1Raw() {
        return slide1Encoder.getDistance();
    }
    
    public double getSlide2Raw() {
        return slide2Encoder.getDistance();
    }
    
    public void setDistancePerPulse(float leftDPP, float rightDPP, float slideDPP) {
        leftEncoder.setDistancePerPulse(leftDPP);
        rightEncoder.setDistancePerPulse(rightDPP);        
        slide1Encoder.setDistancePerPulse(slideDPP);
        slide2Encoder.setDistancePerPulse(slideDPP);
    }
    
    private class UpdateLocation extends Thread {

        public double x;
        public double y;
        public double heading;
        private boolean ResetFlag = false;
        
        UpdateLocation() {
            x = 0;
            y = 0;    
            heading = 0;
            headingGyro.reset();
        }
        
        public void Reset() {
            ResetFlag = true;
        }
        
        public void run() {       
            double deltax, deltay;
            double distance;
            double lastLeft = 0;
            double lastRight = 0;
            double left, right;
            double slide1, slide2;
            double lastSlide1 = 0;
            double lastSlide2 = 0;
    		long startTime; 
    		
            
            while (true) {
            	startTime = System.nanoTime();
            	
            	if (ResetFlag) {
                	x = 0;
                    y = 0;
                    headingGyro.reset();
                    leftEncoder.reset();
                    rightEncoder.reset();
                    ResetFlag = false;
                }
                left = leftEncoder.getDistance();
                right = rightEncoder.getDistance();
                
                /* Average the two encoder values */
                /* TODO - posiably add error checking to detect a failed encoder and ignore it */
                distance = ((left - lastLeft) + (right - lastRight)) / 2.0;
                
                heading = headingGyro.GetHeading();

                /* Do fancy trig stuff */
                deltax = distance * Math.cos(Math.toRadians(heading));
                deltay = distance * Math.sin(Math.toRadians(heading));
                
                /* Factor in slide drive */
                slide1 = slide1Encoder.getDistance();
                slide2 = slide2Encoder.getDistance();  //there is no slide2 on practice bot
                //TODO - average these?
                deltax += (lastSlide1-slide1) * Math.sin(Math.toRadians(heading));
                deltay += (lastSlide1-slide1) * Math.cos(Math.toRadians(heading));

                /* Update Location */
                x += deltax;
                y -= deltay;

                /* Update history variables */
                lastLeft = left;
                lastRight = right;
                lastSlide1 = slide1;
                lastSlide2 = slide2;
                
                SmartDashboard.putNumber("x", x);
                SmartDashboard.putNumber("y", y);
                SmartDashboard.putNumber("Heading", heading);
                SmartDashboard.putNumber("left", left);
                SmartDashboard.putNumber("right", right);
                SmartDashboard.putNumber("slide1", slide1);
                SmartDashboard.putNumber("slide2", slide2);

               	try {
                	Thread.sleep((System.nanoTime() - startTime)/1000000  + 10); //Update the position at 10ms (100Hz) 
                } catch (InterruptedException e) {
                }
            }
        }    
    }

	public double getLeftEncoderDistance() {
		return leftEncoder.getDistance();
	}
	
	public double getRightEncoderDistance() {
		return rightEncoder.getDistance();
	}

	public double getSlide1EncoderDistance() {
		return slide1Encoder.getDistance();
	}
	
	public double getSlide2EncoderDistance() {
		return slide2Encoder.getDistance();
	}
	
	public double getGyroAngleInRadians() {
		return updateLocation.heading * Math.PI/180.0;
	}

	public double GetVel() {
		return leftEncoder.getRate(); // distance/second
	}

	public double GetHeadingRadians() {
		return getGyroAngleInRadians();
	}

	@Override
	public String GetColumNames() {
		return "x, y, heading";
	}

	@Override
	public String log() {
		return String.format("%.2f,%.2f,%.2f", updateLocation.x, updateLocation.y, updateLocation.heading);
	}
}
