/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  boolean isfieldcentric = false;
  WPI_TalonSRX leftmaster;
  WPI_TalonSRX leftslave;
  WPI_TalonSRX rightmaster;
  WPI_TalonSRX rightslave;
  WPI_TalonSRX centermaster;
  WPI_TalonSRX centerslave;
  AHRS navx;
  DifferentialDrive RobotDrive;
  StickyFaults centermasterfaults;
  //boolean centerpresent = (centermaster.getStickyFaults(centermasterfaults) == ErrorCode.OK);

  public drivetrain() {
    navx = new AHRS(Port.kMXP, (byte) 200);
    leftmaster = new WPI_TalonSRX(RobotMap.ct_left_1);
    leftslave = new WPI_TalonSRX(RobotMap.ct_left_2);
    rightmaster = new WPI_TalonSRX(RobotMap.ct_right_1);
    rightslave = new WPI_TalonSRX(RobotMap.ct_right_2);
    centermaster = new WPI_TalonSRX(RobotMap.ct_center_1);
    centerslave = new WPI_TalonSRX(RobotMap.ct_center_2);
    centerslave.set(ControlMode.Follower, centermaster.getDeviceID());
    leftslave.set(ControlMode.Follower, leftmaster.getDeviceID());
    rightslave.set(ControlMode.Follower, rightmaster.getDeviceID());

    RobotDrive = new DifferentialDrive(leftmaster, rightmaster);

  }
  
  public void alldrive(double throttle, double rotation, double strafe){
    if (isfieldcentric){
      RobotDrive.arcadeDrive(throttle * Math.cos(getyaw()) + strafe * Math.sin(getyaw()), -rotation);
      centermaster.set(ControlMode.PercentOutput, (-throttle * Math.sin(getyaw()) + strafe * Math.cos(getyaw())));
    }
    else{
      RobotDrive.arcadeDrive(throttle, -rotation);
      centermaster.set(ControlMode.PercentOutput, strafe);
    }
    SmartDashboard.putNumber("yaw", getyaw());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    navx.zeroYaw();
   }

  public boolean isfieldcentric(){
    return isfieldcentric;
  }

  public void setfieldcentric(Boolean set){
    isfieldcentric = set;
  }

  public double getyaw(){
    return navx.getYaw() / 180 * Math.PI;
  }
}
