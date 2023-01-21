/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class CommandDriveDistance extends CommandBase {
  /**
   * Creates a new CommandDriveDistance.
   */
  private final BaseSubsystem m_baseSubsystem;
  private final double distance;

  public CommandDriveDistance(BaseSubsystem baseSubsystem, double position) {
    m_baseSubsystem = baseSubsystem;
    distance = position * 5.1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_baseSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_baseSubsystem.driveToTarget(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
    m_baseSubsystem.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("error", -m_baseSubsystem.getDistanceRight() - distance);
    return Math.abs(-m_baseSubsystem.getDistanceRight() - distance) < 0.25 && m_baseSubsystem.getVelocityLeft() < 1;
  }
}
