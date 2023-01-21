/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.ClimberSubsystem;

public class CommandRunClimberMotor extends CommandBase {
  /**
   * Creates a new CommandRunClimberWench.
   */
  private ClimberSubsystem m_climberSubsystem;
  private Xbox m_controller;

  public CommandRunClimberMotor(ClimberSubsystem climberSubsystem, Xbox controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_climberSubsystem.runWinch(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.runMotor(m_controller.getRightT() - m_controller.getLeftT());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
