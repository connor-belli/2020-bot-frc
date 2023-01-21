/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CompConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */
  private final WPI_TalonSRX motorClimber;
  private final DigitalInput hookLimit;
  private final Solenoid solClimber;

  public ClimberSubsystem() {
    hookLimit = new DigitalInput(kClimberLimit);
    motorClimber = new WPI_TalonSRX(kMotorClimb);
    solClimber = new Solenoid(kSolClimb);

    addChild("Hook Limit Switch", hookLimit);
    addChild("Climber Motor", motorClimber);
    addChild("Climber Solenoid", solClimber);
  }

  public void runMotor(double speed) {
    motorClimber.set(speed);
  }

  public boolean getClimberLimit() {
    return !hookLimit.get();
  }

  public void setCylinders(boolean x) {
    solClimber.set(x);
  }

  public void toggleCylinders() {
    setCylinders(!solClimber.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public void log() {
    SmartDashboard.putNumber("Climber Speed", motorClimber.getMotorOutputPercent());
  }
}
