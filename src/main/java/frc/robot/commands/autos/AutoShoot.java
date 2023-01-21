/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandDoNothing;
import frc.robot.commands.CommandToggleIntake;
import frc.robot.commands.hood.CommandAutoAimHood;
import frc.robot.commands.shooter.CommandShootAll;
import frc.robot.commands.shooter.CommandStartShooter;
import frc.robot.commands.shooter.CommandStopShooter;
import frc.robot.commands.turret.CommandAimTurret;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoShoot.
   */
  public AutoShoot(ShooterSubsystem shooterSubsystem,
                   ShooterHoodSubsystem shooterHoodSubsystem,
                   ShooterTurretSubsystem shooterTurretSubsystem,
                   IndexWheelSubsystem indexWheelSubsystem,
                   IntakeSubsystem intakeSubsystem,
                   LimeLightSubsystem limeLightSubsystem, int shots) {
    super(  new CommandToggleIntake(intakeSubsystem),
            new CommandStartShooter(shooterSubsystem),
            new CommandAimTurret(shooterTurretSubsystem, limeLightSubsystem).withTimeout(2),
            new CommandAutoAimHood(shooterHoodSubsystem, limeLightSubsystem).withTimeout(3),
            new CommandShootAll(indexWheelSubsystem, shooterSubsystem, shots),
            new CommandStopShooter(shooterSubsystem),
            new CommandToggleIntake(intakeSubsystem));
  }
}
