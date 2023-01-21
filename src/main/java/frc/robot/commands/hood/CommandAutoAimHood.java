package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ProjectileMath;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

import static frc.robot.Constants.*;

public class CommandAutoAimHood extends CommandBase {
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final ShooterHoodSubsystem m_shooterHoodSubsystem;

  public CommandAutoAimHood(ShooterHoodSubsystem shooterHoodSubsystem, LimeLightSubsystem limeLightSubsystem) {
    m_limeLightSubsystem = limeLightSubsystem;
    m_shooterHoodSubsystem = shooterHoodSubsystem;
    addRequirements(shooterHoodSubsystem);
  }

  @Override
  public void initialize() {
    m_limeLightSubsystem.useVision(true);
  }

  private boolean useLinearFormula = true;
  @Override
  public void execute() {
    double distance = m_limeLightSubsystem.estimateDistanceAlt();
    if(useLinearFormula) {
      try {
        double x = distance*-13.719 + 44.681;
        if (useLinearFormula) {
          m_shooterHoodSubsystem.setTargetAngle(x);
        }
      } catch(Exception e) {
        

      }
    } else {
      distance = distance / 0.0254;
      m_shooterHoodSubsystem.setTargetAngle(ProjectileMath.getAngle(distance) - 6.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    //m_shooterHoodSubsystem.setTargetAngle(kShooterBottomLimitAngle);
    m_limeLightSubsystem.useVision(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
