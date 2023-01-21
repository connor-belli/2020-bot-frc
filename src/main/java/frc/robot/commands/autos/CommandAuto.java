/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Xbox;
import frc.robot.commands.base.CommandDriveDistanceBuilder;
import frc.robot.commands.base.CommandDriveUntilWall;
import frc.robot.commands.base.CommandTurnBuilder;
import frc.robot.commands.shooter.CommandAlignToTarget;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CommandAuto extends SequentialCommandGroup {
  /**
   * Creates a new CommandAuto.
   */

  public CommandAuto(BaseSubsystem baseSubsystem, LimeLightSubsystem limeLightSubsystem, Xbox controller) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
    CommandDriveDistanceBuilder drive = new CommandDriveDistanceBuilder(baseSubsystem);
    CommandTurnBuilder turn = new CommandTurnBuilder(baseSubsystem);
    addCommands(
            drive.build(5),
            turn.build(90),
            new CommandDriveUntilWall(baseSubsystem),
            drive.build(-1),
            turn.build(90),
            new CommandAlignToTarget(baseSubsystem, limeLightSubsystem, controller));
  }
}
