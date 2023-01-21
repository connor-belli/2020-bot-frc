/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class CommandDriveUntilWall extends CommandBase {
  /**
   * Creates a new CommandDriveUntilWall.
   */
  private final BaseSubsystem m_baseSubsystem;

  public CommandDriveUntilWall(BaseSubsystem baseSubsystem) {
    m_baseSubsystem = baseSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_baseSubsystem.isDistanceValid()) {
      m_baseSubsystem.arcadeDrive(1, 0);
    } else {
      m_baseSubsystem.arcadeDrive(0.5, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return m_baseSubsystem.getDistance() < 5;
  }
}
