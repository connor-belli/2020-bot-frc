/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private WPI_TalonFX motorRight, motorLeft;
  private double lastVelocity, lastAcceleration, lastJerk;
  private int shotCount;

  public ShooterSubsystem() {
    motorRight = new WPI_TalonFX(kMotorRightShooter);
    motorLeft = new WPI_TalonFX(kMotorLeftShooter);
    motorRight.configFactoryDefault();
    motorLeft.configFactoryDefault();
    motorLeft.setNeutralMode(NeutralMode.Coast);

    motorRight.setInverted(true);
    motorLeft.setInverted(false);
    //motorRight.configOpenloopRamp(0.75);
    //motorLeft.configOpenloopRamp(0.75);

    //motorLeft.enableVoltageCompensation(false);
    //motorRight.enableVoltageCompensation(false);
    //motorLeft.configVoltageCompSaturation(11.5);
    //motorRight.configVoltageCompSaturation(11.5);

    setP(kShooterP);
    setI(kShooterI);
    setD(kShooterD);
    setF(kShooterF);

    motorLeft.follow(motorRight);
    lastVelocity = 0;
    lastAcceleration = 0;
    setShotCount(0);
    addChild("Shooter", motorRight);
    addChild("Shooter", motorLeft);
  }

  public void setP(double p) {
    motorRight.config_kP(0, p);
    motorLeft.config_kP(0, p);
  }

  public void setI(double i) {
    motorRight.config_kI(0, i);
    motorLeft.config_kI(0, i);
  }

  public void setD(double d) {
    motorRight.config_kD(0, d);
    motorLeft.config_kD(0, d);
  }

  public void setF(double f) {
    motorRight.config_kF(0, f);
    motorLeft.config_kF(0, f);
  }

  public void runShooter(double speed) {
    motorRight.set(ControlMode.PercentOutput, speed);
  }

  public void runVelocity(double v) {
    motorRight.set(ControlMode.Velocity, v);
  }

  public void runAmperage(double a) {
    motorRight.set(ControlMode.Current, a);
  }

  public double getVelocity() {
    // velocity is clicks per 100ms
    return motorRight.getSensorCollection().getIntegratedSensorVelocity() * 10 * 60 / 2048;
  }

  public double getVelocityAlt() {
    // velocity is clicks per 100ms
    return motorRight.getSensorCollection().getIntegratedSensorVelocity() * 10 * 60 / 2048;
  }

  public boolean isRunning() {
    return Math.abs(motorRight.getMotorOutputPercent()) > 0.1;
  }

  double bangBangTarget = 0;

  public void runBangBang(double v) {
    bangBangTarget = v;
  }
  public boolean useBangBang = true;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double velocity = getVelocity();
    double acceleration = (velocity - lastVelocity) / 0.02;
    double jerk = (acceleration - lastAcceleration) / 0.02;
    log();
    if (jerk < -50 && jerk < lastJerk) {
      shotCount++;
    }
    lastVelocity = velocity;
    lastAcceleration = acceleration;
    lastJerk = jerk;
  }

  public void log() {
    SmartDashboard.putNumber("Shooter - Speed Right", Math.abs(getVelocity()));
    SmartDashboard.putNumber("Shooter - Speed Left", Math.abs(getVelocityAlt()));
    SmartDashboard.putNumber("Balls Shot", shotCount);
    SmartDashboard.putNumber("Bang bang", bangBangTarget);
    SmartDashboard.putNumber("Shooter - CLE", motorRight.getClosedLoopError());
  }

  public int getShotCount() {
    return shotCount;
  }

  public void setShotCount(int shotCount) {                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    this.shotCount = shotCount;
  }
}

