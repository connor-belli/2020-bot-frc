/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xbox;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class CommandRunTurret extends CommandBase {
  /**
   * Creates a new CommandRunTurret.
   */
  private final ShooterTurretSubsystem m_shooterTurretSubsystem;
  private final Xbox m_controller;

  public CommandRunTurret(ShooterTurretSubsystem shooterTurretSubsystem, Xbox controller) {
    m_shooterTurretSubsystem = shooterTurretSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterTurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = -m_controller.getRightX();
    SmartDashboard.putNumber("turret target", position);
    m_shooterTurretSubsystem.setTargetAngle(m_shooterTurretSubsystem.getTargetAngle() + position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterTurretSubsystem.runTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
