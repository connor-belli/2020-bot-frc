/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class CommandToggleIntake extends InstantCommand {
  public CommandToggleIntake(IntakeSubsystem intakeSubsystem) {
    super(() -> {
      intakeSubsystem.toggleIntake();
      if (!intakeSubsystem.getIntake()) {
        intakeSubsystem.runIntake(1);
      } else {
        intakeSubsystem.runIntake(0);
      }
    }, intakeSubsystem);
  }
}
