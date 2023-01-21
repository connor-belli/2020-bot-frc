/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CompConstants.*;
import static frc.robot.Constants.kMotorControlPanel;

public class ControlPanelSubsystem extends SubsystemBase {
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private Solenoid solPanel;
  private int timesCompleted;
  private ColorSensorV3 colorSensor;
  private WPI_TalonSRX motor;

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {
    motor = new WPI_TalonSRX(kMotorControlPanel);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    solPanel = new Solenoid(kSolWheel);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    timesCompleted = 0;
    addChild("Motor", motor);
  }

  public void runControlPanel(double speed) {
    motor.set(speed);
  }

  public String getColor() {
    String colorString = "Unknown";

    ColorMatchResult match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    }

    return colorString;
  }

  @Override
  public void periodic() {
    log();
  }

  public void toggleSpinner() {
    solPanel.set(!solPanel.get());
  }

  public void log() {
    SmartDashboard.putString("CPanel - Current", getColor());
    SmartDashboard.putBoolean("Cpanel - Motor", motor.get() != 0.0);
  }
}
