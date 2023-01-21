/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class CommandAlignToTarget extends CommandBase {
  /**
   * Creates a new CommandAlignToTarget.
   */
  private final BaseSubsystem m_baseSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final Xbox m_controller;
  private boolean usingVision;

  public CommandAlignToTarget(BaseSubsystem baseSubsystem, LimeLightSubsystem limeLightSubsystem, Xbox controller) {

    m_baseSubsystem = baseSubsystem;
    m_limeLightSubsystem = limeLightSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    usingVision = m_limeLightSubsystem.usingVision();
    m_limeLightSubsystem.useVision(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Math.tanh(-m_limeLightSubsystem.getXAngle() / 25);
    m_baseSubsystem.arcadeDrive(m_controller.getLeftY(), Math.signum(speed) * 0.1 + speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.arcadeDrive(0, 0);
    m_limeLightSubsystem.useVision(usingVision);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limeLightSubsystem.getXAngle()) < 0.5;
  }
}
