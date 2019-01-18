/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid solenoidleft;
  private DoubleSolenoid solenoidright;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public intake(){
    solenoidleft = new DoubleSolenoid(RobotMap.pcm_id, 1, 2);
    solenoidright = new DoubleSolenoid(RobotMap.pcm_id, 3, 4);
  }

  public void setpistons(boolean open){
    if (open){
      solenoidleft.set(Value.kReverse);
      solenoidright.set(Value.kReverse);
    }
    else{
      solenoidleft.set(Value.kForward);
      solenoidright.set(Value.kForward);
    }
  }
}
