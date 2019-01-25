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
  private DoubleSolenoid armsolenoid;
  private DoubleSolenoid kickersolenoid;
  boolean open = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public intake(){
    armsolenoid = new DoubleSolenoid(RobotMap.pcm_id, 1, 0);
    kickersolenoid = new DoubleSolenoid(RobotMap.pcm_id, 3, 2);
  }

  public void setpistons(boolean open){
    if (open){
      armsolenoid.set(Value.kReverse);
    }
    else{
      armsolenoid.set(Value.kForward);
    }
  }

  public void kicker(){
    if (open){
      kickersolenoid.set(Value.kReverse);
    }
    else{
      kickersolenoid.set(Value.kForward);
    }
  }
}
