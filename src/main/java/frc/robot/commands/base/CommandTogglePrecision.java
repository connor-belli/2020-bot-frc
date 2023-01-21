package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BaseSubsystem;

public class CommandTogglePrecision extends InstantCommand {
  public CommandTogglePrecision(BaseSubsystem baseSubsystem) {
    super(baseSubsystem::toggleInputMultiplier, baseSubsystem);
  }
}
