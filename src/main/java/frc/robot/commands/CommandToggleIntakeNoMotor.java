package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class CommandToggleIntakeNoMotor extends InstantCommand {
  public CommandToggleIntakeNoMotor(IntakeSubsystem intakeSubsystem) {
    super(intakeSubsystem::toggleIntake, intakeSubsystem);
  }
}
