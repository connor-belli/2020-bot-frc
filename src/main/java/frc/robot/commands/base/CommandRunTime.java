/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class CommandRunTime extends CommandBase {
  /**
   * Creates a new CommandRunTime.
   */
  private final BaseSubsystem m_baseSubsystem;
  private final Timer m_timer;
  private final double m_time;
  public CommandRunTime(BaseSubsystem baseSubsystem, double time) {
    m_time = time;
    m_timer = new Timer();
    m_baseSubsystem = baseSubsystem;
    addRequirements(baseSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_baseSubsystem.arcadeDrive(-1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
