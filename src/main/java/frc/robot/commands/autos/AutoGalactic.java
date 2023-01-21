/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.hood.CommandAutoAimHood;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoGalactic extends InstantCommand {
  final LimeLightSubsystem m_limeLightSubsystem;
  final BaseSubsystem m_baseSubsystem;
  final Command ra, rb, ba, bb;
  public AutoGalactic(LimeLightSubsystem limeLightSubsystem, BaseSubsystem baseSubsystem) {
    m_limeLightSubsystem = limeLightSubsystem;
    m_baseSubsystem = baseSubsystem;
    ra = baseSubsystem.getTrajectoryCommand("ra.path");
    rb = baseSubsystem.getTrajectoryCommand("rb.path");
    ba = baseSubsystem.getTrajectoryCommand("ba.path");
    bb = baseSubsystem.getTrajectoryCommand("bb.path");
  }

  // Called when the command is initially scheduled.
  double minTargetSize = 0.04;
  @Override
  public void initialize() {
    if(m_limeLightSubsystem.getNumTargets() == 3) {
      if(m_limeLightSubsystem.getSize() > minTargetSize) {
        ra.schedule();
      } else {
        ba.schedule();
      }
    } else {
      if(m_limeLightSubsystem.getSize() > minTargetSize) {
        rb.schedule();
      } else {
        bb.schedule();
      }
    }
  }
}
