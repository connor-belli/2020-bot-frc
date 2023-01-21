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

import static frc.robot.Constants.kMotorHopper;
import static frc.robot.Constants.kMotorIndex;

public class IndexWheelSubsystem extends SubsystemBase {
  /**
   * Creates a new IndexWheelSubsystem.
   */
  private WPI_TalonSRX motorIndex, motorHopperWheel;

  public IndexWheelSubsystem() {
    motorIndex = new WPI_TalonSRX(kMotorIndex);
    motorHopperWheel = new WPI_TalonSRX(kMotorHopper);
    motorHopperWheel.setInverted(true);
    motorIndex.setInverted(false);

    addChild("Index Motor", motorIndex);
    addChild("HopperWheel Motor", motorHopperWheel);
  }

  public void runIndex(double speed) {
    motorIndex.set(speed);
  }

  public void runHopper(double speed) {
    motorHopperWheel.set(speed);
  }

  @Override
  public void periodic() {
    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Intake - HopperWheel Motor", motorHopperWheel.get() != 0.0);
  }
}
