/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.CommandDriveDistance;
import frc.robot.commands.base.CommandRunTime;
// import frc.robot.commands.base.CommandTurnPID;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootInitLine extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootInitLine.
   */
  public AutoShootInitLine(BaseSubsystem baseSubsystem,
                           ShooterSubsystem shooterSubsystem,
                           ShooterHoodSubsystem shooterHoodSubsystem,
                           ShooterTurretSubsystem shooterTurretSubsystem,
                           IndexWheelSubsystem indexWheelSubsystem,
                           IntakeSubsystem intakeSubsystem,
                           LimeLightSubsystem limeLightSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
            // new CommandTurnPID(baseSubsystem, 180),
            new AutoShoot(shooterSubsystem, shooterHoodSubsystem, shooterTurretSubsystem, indexWheelSubsystem, intakeSubsystem, limeLightSubsystem, 3),
            new CommandRunTime(baseSubsystem, 0.3));
  }
}
