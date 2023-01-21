package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class CommandRetractClimber extends InstantCommand {
    public CommandRetractClimber(ClimberSubsystem climberSubsystem) {
      super(() -> climberSubsystem.setCylinders(false), climberSubsystem);
    }
}
