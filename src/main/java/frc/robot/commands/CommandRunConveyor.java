/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.IndexWheelSubsystem;

public class CommandRunConveyor extends CommandBase {
  /**
   * Creates a new CommandRunConveyor.
   */
  private final IndexWheelSubsystem m_indexWheelSubsystem;
  private final Xbox m_controller;

  public CommandRunConveyor(IndexWheelSubsystem indexWheelSubsystem, Xbox controller) {
    m_indexWheelSubsystem = indexWheelSubsystem;
    m_controller = controller;
    addRequirements(indexWheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_controller.getRightT()) > 0.25) {
      m_indexWheelSubsystem.runHopper(1);
      m_indexWheelSubsystem.runIndex(1);
    } else if(Math.abs(m_controller.getLeftT()) > 0.25) {
      m_indexWheelSubsystem.runHopper(-1);
      m_indexWheelSubsystem.runIndex(-1);
    } else {
      m_indexWheelSubsystem.runHopper(0);
      m_indexWheelSubsystem.runIndex(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexWheelSubsystem.runHopper(0);
    m_indexWheelSubsystem.runIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
