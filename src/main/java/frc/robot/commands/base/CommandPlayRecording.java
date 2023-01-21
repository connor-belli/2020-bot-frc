/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class CommandPlayRecording extends CommandBase {
  /**
   * Creates a new CommandPlayRecording.
   */
  int ind = 0;
  final BaseSubsystem m_baseSubsystem;
  public CommandPlayRecording(BaseSubsystem baseSubsystem) {
    m_baseSubsystem = baseSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ind = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_baseSubsystem.arcadeDrive(m_baseSubsystem.throttle.get(ind), m_baseSubsystem.steer.get(ind));
    ind++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ind >= m_baseSubsystem.steer.size();
  }
}
