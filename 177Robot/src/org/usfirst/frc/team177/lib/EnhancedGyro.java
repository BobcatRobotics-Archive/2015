/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.Gyro;
import org.usfirst.frc.team177.robot.Constants;

/**
 *
 * @author SCHROED
 */
public class EnhancedGyro extends Gyro {
    
    public EnhancedGyro() {
        super(Constants.AIOGyro.getInt());
    }
    
    public double GetHeading() {
        double a =  getAngle()%360;
        if (a < 0) {
            a += 360;
        }
        return a;
    }
}
