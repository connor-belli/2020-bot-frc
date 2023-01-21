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

  }

  private boolean useLinearFormula = false;
  @Override
  public void execute() {
    if (useLinearFormula) {
      double distance = m_limeLightSubsystem.estimateDistanceAlt() / 0.0254;
      double height = (kTargetHeight - kLimeLightHeight) / 0.0254;
      double[] params = ProjectileMath.getOptimumParams(distance, height);
      SmartDashboard.putNumber("Error squared", ProjectileMath.minDist(params[0], params[1], kProjectileConstant, kGravity, distance, height, 0.01));
      SmartDashboard.putNumber("Estimated angle", Math.toDegrees(params[0]));
      m_shooterHoodSubsystem.setTargetAngle(Math.toDegrees(params[0]));
    } else {
      double distance = m_limeLightSubsystem.estimateDistanceAlt();
      m_shooterHoodSubsystem.setTargetAngle(ProjectileMath.getAngle(distance));
    }
  }

  @Override
  public void end(boolean interrupted) {
    //m_shooterHoodSubsystem.setTargetAngle(kShooterBottomLimitAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
