/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.comp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.CompConstants.kSolShift;
import static frc.robot.Constants.*;


public class CompBaseSubsystem extends BaseSubsystem {
  private final WPI_TalonFX motorLeft1, motorLeft2;
  private final WPI_TalonFX motorRight1, motorRight2;
  private final Solenoid solShift;
  private double p = 0, i = 0, d = 0, f = 0;

  public CompBaseSubsystem() {
    this(new WPI_TalonFX(kMotorBaseL1), new WPI_TalonFX(kMotorBaseR1));
  }

  private CompBaseSubsystem(WPI_TalonFX left1, WPI_TalonFX right1) {
    super(left1, right1);
    motorLeft1 = left1;
    setUpMotor(motorLeft1);
    motorLeft2 = new WPI_TalonFX(kMotorBaseL2);
    setUpMotor(motorLeft2);
    motorRight1 = right1;
    setUpMotor(motorRight1);
    motorRight2 = new WPI_TalonFX(kMotorBaseR2);
    setUpMotor(motorRight2);
    motorLeft2.follow(motorLeft1);
    motorRight2.follow(motorRight1);

    solShift = new Solenoid(kSolShift);

    addChild("Base Drive", diffDrive);
    addChild("Base Shifter", solShift);
    // addChild("NavX", ahrs);
    addChild("Distance Sensor", distanceSensor);

    setP(kBaseP);
    setI(kBaseI);
    setD(kBaseD);
    setF(kBaseF);

    SmartDashboard.putNumber("p", SmartDashboard.getNumber("p", p));
    SmartDashboard.putNumber("i", SmartDashboard.getNumber("i", i));
    SmartDashboard.putNumber("d", SmartDashboard.getNumber("d", d));
    SmartDashboard.putNumber("f", SmartDashboard.getNumber("f", f));
  }

  private void setUpMotor(WPI_TalonFX motor) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.5);
    motor.configClosedloopRamp(0.5);
  }

  public void resetEncoder() {
    motorLeft1.getSensorCollection().setIntegratedSensorPosition(0, 5);
    motorRight1.getSensorCollection().setIntegratedSensorPosition(0, 5);
  }

  public void driveToTarget(double target) {
    motorLeft1.set(ControlMode.Position, target);
    motorRight1.set(ControlMode.Position, target);
  }

  @Override
  public void arcadeDrive(double speed, double rotation) {
    super.arcadeDrive(-speed, rotation);
  }
  public void tankDrive(double left, double right) {
    super.tankDrive(-left, -right);
  }

  public double getDistanceLeft() {
    return motorLeft1.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getDistanceRight() {
    return motorRight1.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getVelocityLeft() {
    return motorLeft1.getSensorCollection().getIntegratedSensorVelocity();
  }

  public double getVelocityRight() {
    return motorRight1.getSensorCollection().getIntegratedSensorVelocity();
  }

  public void setP(double p) {
    if (this.p != p) {
      motorLeft1.config_kF(0, p);
      motorLeft2.config_kF(0, p);
      motorRight1.config_kF(0, p);
      motorRight2.config_kF(0, p);
    }
    this.p = p;
  }

  public void setI(double i) {
    if (this.i != i) {
      motorLeft1.config_kI(0, i);
      motorLeft2.config_kI(0, i);
      motorRight1.config_kI(0, i);
      motorRight2.config_kI(0, i);
    }
    this.i = i;
  }

  public void setD(double d) {
    if (this.d != d) {
      motorLeft1.config_kD(0, d);
      motorLeft2.config_kD(0, d);
      motorRight1.config_kD(0, d);
      motorRight2.config_kD(0, d);
    }
    this.d = d;
  }

  public void setF(double f) {
    if (this.f != f) {
      motorLeft1.config_kF(0, f);
      motorLeft2.config_kF(0, f);
      motorRight1.config_kF(0, f);
      motorRight2.config_kF(0, f);
    }
    this.f = f;
  }

  @Override
  public void periodic() {
    super.periodic();
    updatePIDs();
    log();
  }

  private void updatePIDs() {
    final double kP = SmartDashboard.getNumber("p", p),
            kI = SmartDashboard.getNumber("i", i),
            kD = SmartDashboard.getNumber("d", d),
            kF = SmartDashboard.getNumber("f", f);
    setP(kP);
    setI(kI);
    setD(kD);
    setF(kF);
  }

  public void log() {
    SmartDashboard.putString("Base - Shifter", !solShift.get()?"Low gear":"High gear");
  }

  @Override
  public void toggleShift() {
    solShift.set(!solShift.get());
  }
}
