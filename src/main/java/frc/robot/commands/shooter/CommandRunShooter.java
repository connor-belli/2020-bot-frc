/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandRunShooter extends CommandBase {
  private static double shooterSpeed = 0;
  /**
   * Creates a new CommandRunShooter.
   */
  private final ShooterSubsystem m_shooterSubsystem;
  private final Xbox m_controller;

  public CommandRunShooter(ShooterSubsystem shooterSubsystem, Xbox controller) {
    m_shooterSubsystem = shooterSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double trigger = m_controller.getRightT();
    if (trigger > 0.25) {
      SmartDashboard.putNumber("trigger", shooterSpeed);
      m_shooterSubsystem.runShooter(1);
    } else {
      m_shooterSubsystem.runShooter(0);
    }

    double projectileSpeed = (m_shooterSubsystem.getVelocity() * 0.0762 * 2 * 3.14 / 60) / 1.5;
    SmartDashboard.putNumber("Speed", m_shooterSubsystem.getVelocity());
    //double dist = ProjectileMath.dist(Math.PI/3, projectileSpeed, kProjectileConstant, kGravity);
    //SmartDashboard.putNumber("Estimated Distance", dist);
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
