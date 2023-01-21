package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.CommandDriveDistance;
import frc.robot.subsystems.*;

public class AutoPickupFromPartner extends SequentialCommandGroup {
  public AutoPickupFromPartner(BaseSubsystem baseSubsystem,
                               ShooterSubsystem shooterSubsystem,
                               ShooterHoodSubsystem shooterHoodSubsystem,
                               ShooterTurretSubsystem shooterTurretSubsystem,
                               IndexWheelSubsystem indexWheelSubsystem,
                               LimeLightSubsystem limeLightSubsystem,
                               IntakeSubsystem intakeSubsystem) {
    super(new AutoShoot(shooterSubsystem, shooterHoodSubsystem, shooterTurretSubsystem, indexWheelSubsystem, intakeSubsystem, limeLightSubsystem, 3),
            new CommandDriveDistance(baseSubsystem, -4),
            new AutoShoot(shooterSubsystem, shooterHoodSubsystem, shooterTurretSubsystem, indexWheelSubsystem, intakeSubsystem, limeLightSubsystem, 3));
  }
}
