/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.Gyro;

/**
 *
 * @author SCHROED
 */
public class EnhancedGyro extends Gyro {
    
    public EnhancedGyro(int channel) {
        super(channel);
    }
    
    public double GetHeading() {
        double a =  getAngle()%360;
        if (a < 0) {
            a += 360;
        }
        return a;
    }
}
