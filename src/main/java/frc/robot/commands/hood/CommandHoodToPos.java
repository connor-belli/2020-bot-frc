package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class CommandHoodToPos extends InstantCommand {
  public CommandHoodToPos(ShooterHoodSubsystem shooterHoodSubsystem, double position) {
    super(() -> shooterHoodSubsystem.setTargetAngle(position), shooterHoodSubsystem);
  }
}
