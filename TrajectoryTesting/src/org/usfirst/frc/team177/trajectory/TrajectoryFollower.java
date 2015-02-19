package org.usfirst.frc.team177.trajectory;


//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Blatantly stolen for ChezzyPoofs 2014 code

/**
 * PID + Feedforward controller for following a Trajectory.
 *
 * @author Jared341
 */
public class TrajectoryFollower {

  private double kp_;
  private double ki_;  // Not currently used, but might be in the future.
  private double kd_;
  private double kv_;
  private double ka_;
  private double last_error_;
  private double last_distance_so_far_;
  private double current_heading = 0;
  private int current_segment;
  private Trajectory profile_;
  public String name;

  public TrajectoryFollower(String name) {
    this.name = name;
  }

  public void configure(double kp, double ki, double kd, double kv, double ka) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kv_ = kv;
    ka_ = ka;
  }

  public void reset() {
    last_error_ = 0.0;
    current_segment = 0;
  }

  public void setTrajectory(Trajectory profile) {
    profile_ = profile;
  }

  public double calculate(double distance_so_far) {
	last_distance_so_far_ = distance_so_far;
    if (current_segment < profile_.getNumSegments()) {
      Trajectory.Segment segment = profile_.getSegment(current_segment);
      double error = segment.pos - distance_so_far;
      double output = kp_ * error + kd_ * ((error - last_error_)
              / segment.dt - segment.vel) + (kv_ * segment.vel
              + ka_ * segment.acc);

      last_error_ = error;
      current_heading = segment.heading;
      current_segment++;
      return output;
    } else {
      return 0;
    }
  }

  //Used for simulating and logging
  public double getExpectedDistance() {
	  return profile_.getSegment(current_segment).pos;
  }
  
  public double getHeading() {
    return current_heading;
  }

  public boolean isFinishedTrajectory() {
    return current_segment >= profile_.getNumSegments();
  }
  
  public int getCurrentSegment() {
    return current_segment;
  }
  
  public int getNumSegments() {
    return profile_.getNumSegments();
  }

	public String GetColumNames() {
		return "last_error, current_segment, segment_distance, segment_vel, segment_accel, last_distance";
	}
	
	
	public String log() {
		return String.format("%.2f,%d,%.2f,%.2f,%.2f,%.2f", last_error_, current_segment, profile_.getSegment(current_segment).pos, profile_.getSegment(current_segment).vel, profile_.getSegment(current_segment).acc, last_distance_so_far_);
	}
}
