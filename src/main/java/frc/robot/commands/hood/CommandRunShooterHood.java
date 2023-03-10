/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class CommandRunShooterHood extends CommandBase {
  /**
   * Creates a new CommandRunShooterHood.
   */
  private final ShooterHoodSubsystem m_shooterHoodSubsystem;
  private final Xbox m_controller;

  public CommandRunShooterHood(ShooterHoodSubsystem shooterHoodSubsystem, Xbox controller) {
    m_shooterHoodSubsystem = shooterHoodSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterHoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = m_controller.getLeftY();
    SmartDashboard.putNumber("hood target", position);
    m_shooterHoodSubsystem.setTargetAngle(m_shooterHoodSubsystem.getTargetAngle() + position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
