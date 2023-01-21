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
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoRunIntakeDistance extends SequentialCommandGroup {
  /**
   * Creates a new AutoRunIntakeDistance.
   */
  public AutoRunIntakeDistance(BaseSubsystem baseSubsystem, IntakeSubsystem intakeSubsystem, double distance) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new CommandToggleIntake(intakeSubsystem),
            new CommandDriveDistance(baseSubsystem, distance),
            new CommandToggleIntake(intakeSubsystem));
  }
}
