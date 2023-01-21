/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.practice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.PracticeConstants.kSolIntakeA;
import static frc.robot.PracticeConstants.kSolIntakeB;

public class PracticeIntakeSubsystem extends IntakeSubsystem {
  /**
   * Creates a new PracticeIntakeSubsystem.
   */
  private DoubleSolenoid solIntake;

  public PracticeIntakeSubsystem() {
    super();
    solIntake = new DoubleSolenoid(kSolIntakeA, kSolIntakeB);
    solIntake.set(Value.kReverse);
    SmartDashboard.putNumber("Intake speed", 1);
  }

  public boolean getIntake() {
    return solIntake.get() == Value.kForward;
  }

  public void setIntake(boolean x) {
    solIntake.set(x ? Value.kForward : Value.kReverse);
  }

  @Override
  public void runIntake(double speed) {
    if (speed > 0.1) {
      super.runIntake(SmartDashboard.getNumber("Intake speed", 1));
    } else {
      super.runIntake(0);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
