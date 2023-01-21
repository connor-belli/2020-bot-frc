package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class CommandToggleClimber extends InstantCommand {
  public CommandToggleClimber(ClimberSubsystem climberSubsystem) {
    super(climberSubsystem::toggleCylinders, climberSubsystem);
  }
}
