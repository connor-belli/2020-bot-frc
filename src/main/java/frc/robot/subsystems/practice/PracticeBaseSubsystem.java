/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.practice;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.PracticeConstants.*;

public class PracticeBaseSubsystem extends BaseSubsystem {
  private final CANSparkMax motorLeft1, motorLeft2;
  private final CANSparkMax motorRight1, motorRight2;
  private final DoubleSolenoid solShift;


  public PracticeBaseSubsystem() {
    this(new CANSparkMax(kMotorBaseL1, MotorType.kBrushless), new CANSparkMax(kMotorBaseR1, MotorType.kBrushless));
  }

  private PracticeBaseSubsystem(CANSparkMax left1, CANSparkMax right1) {
    super(left1, right1);
    motorLeft1 = left1;
    setUpMotor(motorLeft1);
    motorLeft2 = new CANSparkMax(kMotorBaseL2, MotorType.kBrushless);
    setUpMotor(motorLeft2);
    motorRight1 = right1;
    setUpMotor(motorRight1);
    motorRight2 = new CANSparkMax(kMotorBaseR2, MotorType.kBrushless);
    setUpMotor(motorRight2);

    motorLeft2.follow(motorLeft1);
    motorRight2.follow(motorRight1);

    solShift = new DoubleSolenoid(kSolShiftA, kSolShiftB);
  }

  private void setUpMotor(CANSparkMax motor) {
    motor.setOpenLoopRampRate(0.5);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void arcadeDrive(double speed, double rotation) {
    double rotMult = Util.clamp((Math.abs(speed)*0.5 + 0.6), 0, 1);
    rotation = 0.5*rotation*(Math.abs(rotation) + 1);
    super.arcadeDrive(speed, rotation*rotMult);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putString("Base - Shifter", String.valueOf(solShift.get()));
  }

  public void driveToTarget(double target) {
    motorLeft1.getPIDController().setReference(target, ControlType.kPosition);
    motorRight1.getPIDController().setReference(-target, ControlType.kPosition);
  }

  public double getDistanceLeft() {
    return motorLeft1.getEncoder().getPosition();
  }

  public double getDistanceRight() {
    return motorRight1.getEncoder().getPosition();
  }

  public double getVelocityLeft() {
    return motorLeft1.getEncoder().getVelocity();
  }

  public double getVelocityRight() {
    return motorRight1.getEncoder().getVelocity();
  }

  @Override
  public void resetEncoder() {
    motorLeft1.getEncoder().setPosition(0);
    motorRight1.getEncoder().setPosition(0);
  }

  @Override
  public void toggleShift() {
    if (solShift.get() == Value.kForward) {
      solShift.set(Value.kReverse);
    } else {
      solShift.set(Value.kForward);
    }
  }

  @Override
  public void setShift(boolean x) {
    if (x) {
      solShift.set(Value.kReverse);
    } else {
      solShift.set(Value.kForward);
    }
  }
}
