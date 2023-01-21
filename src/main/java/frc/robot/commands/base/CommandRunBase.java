/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.BaseSubsystem;

public class CommandRunBase extends CommandBase {
  private final BaseSubsystem m_base;
  private final Xbox m_controller;

  public CommandRunBase(BaseSubsystem m_baseSubsystem, Xbox controller) {
    m_base = m_baseSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_base.getDriveStyle()) {
      case SINGLE_ARCADE:
        m_base.arcadeDrive(-m_controller.getLeftY(), -m_controller.getLeftX());
        break;
      case TANK_DRIVE:
        m_base.tankDrive(-m_controller.getRightY(), -m_controller.getLeftY());
        break;
      case TRIGGER_ARCADE:
        m_base.arcadeDrive(m_controller.getRightT() - m_controller.getLeftT(), -m_controller.getLeftX());
        break;
      default:
        m_base.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
        if (m_base.recording) {
          m_base.steer.add(new Float(-m_controller.getRightX()));
          m_base.throttle.add(new Float(-m_controller.getLeftY()));
        }
        break;
    }
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
