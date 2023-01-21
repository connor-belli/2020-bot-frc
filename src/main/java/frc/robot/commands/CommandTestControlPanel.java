package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class CommandTestControlPanel extends CommandBase {
  private final ControlPanelSubsystem m_controlPanelSubsystem;

  public CommandTestControlPanel(ControlPanelSubsystem controlPanelSubsystem) {
    m_controlPanelSubsystem = controlPanelSubsystem;
    addRequirements(controlPanelSubsystem);
  }

  @Override
  public void initialize() {
    m_controlPanelSubsystem.runControlPanel(1);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_controlPanelSubsystem.runControlPanel(0);
  }
}
