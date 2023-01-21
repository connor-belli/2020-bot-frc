/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class CommandAimTurret extends CommandBase {
  /**
   * Creates a new CommandRunTurret.
   */
  private final ShooterTurretSubsystem m_shooterTurretSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private boolean usingVision = false;

  public CommandAimTurret(ShooterTurretSubsystem shooterTurretSubsystem, LimeLightSubsystem limeLightSubsystem) {
    m_shooterTurretSubsystem = shooterTurretSubsystem;
    m_limeLightSubsystem = limeLightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterTurretSubsystem);
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
    double angleRelative = m_limeLightSubsystem.getXAngle();
    SmartDashboard.putNumber("turret target", angleRelative);
    m_shooterTurretSubsystem.setTargetAngle(m_shooterTurretSubsystem.getEncoderAngle() - angleRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLightSubsystem.useVision(usingVision);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
