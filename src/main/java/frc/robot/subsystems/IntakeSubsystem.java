/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CompConstants.kMotorIntake;

public abstract class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX motorIntake;

  public IntakeSubsystem() {
    motorIntake = new WPI_TalonSRX(kMotorIntake);
    motorIntake.setInverted(true);

    addChild("Intake Motor", motorIntake);
  }

  public void runIntake(double speed) {
    motorIntake.set(speed);
  }

  public void toggleIntake() {
    setIntake(!getIntake());
  }

  public abstract boolean getIntake();

  public abstract void setIntake(boolean x);

  @Override
  public void periodic() {
    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Intake - Intake Motor", motorIntake.get() != 0.0);
    SmartDashboard.putBoolean("Intake Solenoid", getIntake());
  }
}
