/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandToggleIntake;
import frc.robot.commands.base.CommandDriveDistance;
// import frc.robot.commands.base.CommandTurnPID;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTrenchRunAlliance extends SequentialCommandGroup {
  /**
   * Creates a new AutoTrenchRunAlliance.
   */
  public AutoTrenchRunAlliance(BaseSubsystem baseSubsystem,
                               ShooterSubsystem shooterSubsystem,
                               ShooterHoodSubsystem shooterHoodSubsystem,
                               ShooterTurretSubsystem shooterTurretSubsystem,
                               IndexWheelSubsystem indexWheelSubsystem,
                               LimeLightSubsystem limeLightSubsystem,
                               IntakeSubsystem intakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoShoot(shooterSubsystem, shooterHoodSubsystem, shooterTurretSubsystem, indexWheelSubsystem, limeLightSubsystem, 3),
            new CommandDriveDistance(baseSubsystem, -4),
            // new CommandTurnPID(baseSubsystem, 90),
            new CommandDriveDistance(baseSubsystem, 4),
            // new CommandTurnPID(baseSubsystem, 90),
            new CommandToggleIntake(intakeSubsystem),
            new CommandDriveDistance(baseSubsystem, 19),
            new CommandToggleIntake(intakeSubsystem));
  }
}
