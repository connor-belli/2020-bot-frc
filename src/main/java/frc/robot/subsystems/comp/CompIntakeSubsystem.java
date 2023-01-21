/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.comp;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.CompConstants.kSolIntake;

public class CompIntakeSubsystem extends IntakeSubsystem {
  private final Solenoid solIntake;

  public CompIntakeSubsystem() {
    super();
    solIntake = new Solenoid(kSolIntake);
    solIntake.set(false);

    addChild("Intake Solenoid", solIntake);
  }

  @Override
  public boolean getIntake() {
    return solIntake.get();
  }

  @Override
  public void setIntake(boolean x) {
    solIntake.set(x);
  }
}
