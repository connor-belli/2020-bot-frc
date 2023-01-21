/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ShooterTurretSubsystem extends SubsystemBase {
  final private double cpd = -0.0126227208976157;
  /**
   * Creates a new ShooterTurretSubsystem.
   */
  private WPI_TalonSRX motorTurret;
  private DigitalInput limitLeft, limitRight;
  private double p = 0, i = 0, d = 0, f = 0;

  public ShooterTurretSubsystem() {
    motorTurret = new WPI_TalonSRX(kMotorTurret);
    motorTurret.setInverted(false);
    limitLeft = new DigitalInput(kTurretLimitLeft);
    limitRight = new DigitalInput(kTurretLimitRight);

    motorTurret.getSensorCollection().setQuadraturePosition(0, 10);

    setP(kTurretP);
    setI(kTurretI);
    setD(kTurretD);
    setF(kTurretF);

    SmartDashboard.putNumber("turret p", SmartDashboard.getNumber("turret p", p));
    SmartDashboard.putNumber("turret i", SmartDashboard.getNumber("turret i", i));
    SmartDashboard.putNumber("turret d", SmartDashboard.getNumber("turret d", d));
  }

  private double clicksToDegrees(double clicks) {
    return clicks * cpd;
  }

  private int degreesToClicks(double angle) {
    return (int) (angle / cpd);
  }

  public void runTurret(double speed) {
    if ((speed < 0 && !getLimitLeft()) || (speed > 0 && !getLimitRight())) {
      motorTurret.set(speed);
    } else {
      motorTurret.set(0);
    }
  }

  public double getTargetAngle() {
    return clicksToDegrees(motorTurret.getClosedLoopTarget(0));
  }

  public void setTargetAngle(double angle) {
    if (angle < kTurretLeftLimitAngle && getLimitLeft()) angle = kTurretLeftLimitAngle;
    if (angle > kTurretRightLimitAngle && getLimitRight()) angle = kTurretRightLimitAngle;
    motorTurret.set(ControlMode.Position, degreesToClicks(angle));
  }

  public boolean getLimitLeft() {
    return limitLeft.get();
  }

  public boolean getLimitRight() {
    return limitRight.get();
  }

  public boolean isInSafezone() {
    return !getLimitLeft() && !getLimitRight();
  }

  public double getEncoderAngle() {
    return clicksToDegrees(motorTurret.getSensorCollection().getQuadraturePosition());
  }

  public void setEncoderAngle(double angle) {
    motorTurret.getSensorCollection().setQuadraturePosition(degreesToClicks(angle), 10);
  }

  public void setP(double p) {
    if (p != this.p) {
      motorTurret.config_kP(0, p);
    }
    this.p = p;
  }

  public void setI(double i) {
    if (i != this.i) {
      motorTurret.config_kP(0, i);
    }
    this.i = i;
  }

  public void setD(double d) {
    if (d != this.d) {
      motorTurret.config_kP(0, d);
    }
    this.d = d;
  }

  public void setF(double f) {
    if (f != this.f) {
      motorTurret.config_kP(0, f);
    }
    this.f = f;
  }

  @Override
  public void periodic() {
    updatePID();

    if (getLimitLeft()) {
      setEncoderAngle(kTurretLeftLimitAngle);
      if (getTargetAngle() < kTurretLeftLimitAngle)
        setTargetAngle(kTurretLeftLimitAngle);
    }
    if (getLimitRight()) {
      setEncoderAngle(kTurretRightLimitAngle);
      if (getTargetAngle() > kTurretRightLimitAngle)
        setTargetAngle(kTurretRightLimitAngle);
    }
    log();
  }

  private void updatePID() {
    final double kP = SmartDashboard.getNumber("turret p", p),
            kI = SmartDashboard.getNumber("turret i", i),
            kD = SmartDashboard.getNumber("turret d", d);
    setP(kP);
    setI(kI);
    setD(kD);
  }

  public void log() {
    SmartDashboard.putNumber("Turret position", getEncoderAngle());
    SmartDashboard.putBoolean("Turret Limit Left", getLimitLeft());
    SmartDashboard.putBoolean("Turret Limit Right", getLimitRight());
  }
}
