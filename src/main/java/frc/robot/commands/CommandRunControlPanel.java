/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class CommandRunControlPanel extends CommandBase {
  private final ControlPanelSubsystem m_controlPanelSubsystem;
  private boolean isFinished = false;
  private String lastColor;
  private int timesSeen;
  private String gameData;
  private String targetColor;
  private String initialColor;

  /**
   * Creates a new CommandRunControlPanel.
   */
  public CommandRunControlPanel(ControlPanelSubsystem controlPanelSubsystem) {
    m_controlPanelSubsystem = controlPanelSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_controlPanelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    timesSeen = 0;
    isFinished = false;
    initialColor = m_controlPanelSubsystem.getColor();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          targetColor = "Red";
          break;
        case 'G':
          targetColor = "Yellow";
          break;
        case 'R':
          targetColor = "Blue";
          break;
        case 'Y':
          targetColor = "Green";
          break;
        default:
          isFinished = true;
          break;
      }
      lastColor = m_controlPanelSubsystem.getColor();
      m_controlPanelSubsystem.runControlPanel(.5);
    } else {
      if (initialColor == "Unknown") {
        isFinished = true;
      } else {
        lastColor = initialColor;
        m_controlPanelSubsystem.runControlPanel(.5);
      }
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (gameData.length() > 0) {
      String currentColor = m_controlPanelSubsystem.getColor();
      if (currentColor != lastColor) {
        lastColor = currentColor;
        if (lastColor == targetColor) {
          isFinished = true;
        }
      }
    } else {
      String currentColor = m_controlPanelSubsystem.getColor();
      if (currentColor != lastColor) {
        lastColor = currentColor;
        if (lastColor == initialColor) {
          timesSeen++;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelSubsystem.runControlPanel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (gameData.length() > 0) {
      return isFinished;
    } else {
      if (timesSeen > 6) {
        isFinished = true;
      }
      return isFinished;
    }
  }
}

