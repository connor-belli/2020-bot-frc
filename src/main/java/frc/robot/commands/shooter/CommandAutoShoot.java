/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandAutoShoot extends CommandBase {
  /**
   * Creates a new CommandAutoShoot.
   */
  private final ShooterHoodSubsystem m_shooterHoodSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;

  public CommandAutoShoot(ShooterHoodSubsystem shooterHoodSubsystem, ShooterSubsystem shooterSubsystem, LimeLightSubsystem limeLightSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_shooterHoodSubsystem = shooterHoodSubsystem;
    m_limeLightSubsystem = limeLightSubsystem;
    addRequirements(m_shooterSubsystem, m_shooterHoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
