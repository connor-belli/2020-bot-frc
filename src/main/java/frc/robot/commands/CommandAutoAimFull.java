package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class CommandAutoAimFull extends CommandBase {
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final ShooterTurretSubsystem m_shooterTurretSubsystem;
  private final ShooterHoodSubsystem m_shooterHoodSubsystem;
  private Timer timer;
  private double dist;
  private double angle;

  public CommandAutoAimFull(ShooterTurretSubsystem shooterTurretSubsystem, ShooterHoodSubsystem shooterHoodSubsystem, LimeLightSubsystem limeLightSubsystem) {
    m_limeLightSubsystem = limeLightSubsystem;
    m_shooterTurretSubsystem = shooterTurretSubsystem;
    m_shooterHoodSubsystem = shooterHoodSubsystem;
    addRequirements(shooterHoodSubsystem, shooterTurretSubsystem);
  }

  @Override
  public void initialize() {

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

  }
}
