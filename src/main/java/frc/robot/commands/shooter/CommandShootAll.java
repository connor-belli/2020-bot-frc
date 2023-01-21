/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandShootAll extends CommandBase {
  /**
   * Creates a new CommandShootAll.
   */
  private final IndexWheelSubsystem m_indexWheelSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private int shots;
  private Timer timer;

  public CommandShootAll(IndexWheelSubsystem indexWheelSubsystem, ShooterSubsystem shooterSubsystem, int shots) {
    m_indexWheelSubsystem = indexWheelSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_indexWheelSubsystem);
    this.shots = shots;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_shooterSubsystem.getVelocity()) > 5500.0) {
      m_indexWheelSubsystem.runHopper(0.25);
      m_indexWheelSubsystem.runIndex(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 6;//m_shooterSubsystem.getShotCount() == shots;
  }
}
