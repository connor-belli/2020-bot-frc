/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class CommandTurn extends CommandBase {
  /**
   * Creates a new CommandTurn.
   */
  private final BaseSubsystem m_baseSubsystem;
  private final double rotation;

  public CommandTurn(BaseSubsystem baseSubsystem, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_baseSubsystem = baseSubsystem;
    this.rotation = rotation;
    addRequirements(baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_baseSubsystem.resetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_baseSubsystem.arcadeDrive(0, (rotation - m_baseSubsystem.getAngle()) / 100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//(rotation - m_baseSubsystem.getAngle()) < 5;
  }
}
