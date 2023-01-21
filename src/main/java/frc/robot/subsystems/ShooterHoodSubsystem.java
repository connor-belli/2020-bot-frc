/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ShooterHoodSubsystem extends SubsystemBase {
  final private double cpd = 78.0 / 2131.0;
  /**
   * Creates a new ShooterTurretSubsystem.
   */
  private WPI_TalonSRX motorHood;
  private DigitalInput limitTop, limitBottom;
  private double p = 0, i = 0, d = 0, f = 0;

  public ShooterHoodSubsystem() {
    motorHood = new WPI_TalonSRX(kMotorHood);
    motorHood.setInverted(false);
    motorHood.setNeutralMode(NeutralMode.Brake);
    //motorHood.configAllowableClosedloopError(0, 20);
    limitTop = new DigitalInput(kShooterLimitTop);
    limitBottom = new DigitalInput(kShooterLimitBottom);

    motorHood.getSensorCollection().setQuadraturePosition(degreesToClicks(kShooterTopLimitAngle), 10);

    setP(kShooterHoodP);
    setI(kShooterHoodI);
    setD(kShooterHoodD);
    setF(kShooterHoodF);

    SmartDashboard.putNumber("hood p", SmartDashboard.getNumber("hood p", p));
    SmartDashboard.putNumber("hood i", SmartDashboard.getNumber("hood i", i));
    SmartDashboard.putNumber("hood d", SmartDashboard.getNumber("hood d", d));
    setTargetAngle(kShooterBottomLimitAngle);

    addChild("Motor", motorHood);
    addChild("Top limit", limitTop);
    addChild("Bottom limit", limitBottom);
  }

  public double clicksToDegrees(double clicks) {
    return clicks * cpd;
  }

  public int degreesToClicks(double angle) {
    return (int) (angle / cpd);
  }

  public double getTargetAngle() {
    return clicksToDegrees(motorHood.getClosedLoopTarget(0));
  }

  public void setTargetAngle(double angle) {
    //if (getLimitBottom() && angle > kShooterBottomLimitAngle) angle = kShooterBottomLimitAngle;
    if (angle < kShooterTopLimitAngle) angle = kShooterTopLimitAngle;
    motorHood.set(ControlMode.Position, degreesToClicks(angle));
  }

  public boolean getLimitTop() {
    return false; // !limitTop.get();
  }

  public boolean getLimitBottom() {
    return !limitBottom.get();
  }

  public boolean isInSafezone() {
    return !getLimitTop() && !getLimitBottom();
  }

  public double getEncoderAngle() {
    return clicksToDegrees(motorHood.getSensorCollection().getQuadraturePosition());
  }

  public void setEncoderAngle(double angle) {
    motorHood.getSensorCollection().setQuadraturePosition(degreesToClicks(angle), 10);
  }

  public void setP(double p) {
    if (p != this.p) {
      motorHood.config_kP(0, p);
    }
    this.p = p;
  }

  public void setI(double i) {
    if (i != this.i) {
      motorHood.config_kP(0, i);
    }
    this.i = i;
  }

  public void setD(double d) {
    if (d != this.d) {
      motorHood.config_kP(0, d);
    }
    this.d = d;
  }

  public void setF(double f) {
    if (f != this.f) {
      motorHood.config_kP(0, f);
    }
    this.f = f;
  }

  @Override
  public void periodic() {
    updatePID();

    if (getLimitTop()) {
      //setEncoderAngle(kShooterTopLimitAngle);
      //setTargetAngle(kShooterTopLimitAngle);
      setEncoderAngle(kShooterTopLimitAngle);
      if (getTargetAngle() < kShooterTopLimitAngle) {
        setTargetAngle(kShooterTopLimitAngle);
      }
    }
    if (getLimitBottom()) {
      setEncoderAngle(kShooterBottomLimitAngle);
      if (getTargetAngle() > kShooterBottomLimitAngle) {
        setTargetAngle(kShooterBottomLimitAngle);
      }
    }
    log();
  }

  private void updatePID() {
    final double kP = SmartDashboard.getNumber("hood p", p),
            kI = SmartDashboard.getNumber("hood i", i),
            kD = SmartDashboard.getNumber("hood d", d);
    setP(kP);
    setI(kI);
    setD(kD);
  }

  public void log() {
    SmartDashboard.putNumber("Hood position", getEncoderAngle());
    SmartDashboard.putBoolean("Hood Limit Top", getLimitTop());
    SmartDashboard.putBoolean("Hood Limit Bottom", getLimitBottom());
    SmartDashboard.putNumber("Hood Target", getTargetAngle());
  }
}
