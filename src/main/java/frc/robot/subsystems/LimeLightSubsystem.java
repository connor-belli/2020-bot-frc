/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class LimeLightSubsystem extends SubsystemBase {
  private boolean vision = false;
  private NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  private NetworkTableEntry ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");
  private NetworkTableEntry thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor");
  private NetworkTableEntry tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert");
  private NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  private NetworkTableEntry camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
  private NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");

  public LimeLightSubsystem() {
    useVision(false);
  }


  public void useVision(boolean b) {
    if (b) {
      camMode.setNumber(0);
      ledMode.setNumber(3);
    } else {
      camMode.setNumber(1);
      ledMode.setNumber(1);
    }

    vision = b;
  }

  public boolean usingVision() {
    return vision;
  }

  public void toggleVision() {
    useVision(!vision);
  }

  public double estimateDistance() {
    double size = ta.getDouble(0);
    double ratio = thor.getDouble(0) / tvert.getDouble(0);
    double cost = ratio * kTargetRatio;
    double distance = Math.sqrt(cost / size);
    return distance * kTargetSlope;
  }

  public double estimateDistanceAlt() {
    double angle = getYAngle();
    return (kTargetHeight - kLimeLightHeight) / Math.tan(Math.toRadians(angle + kLimeLightAngle));
  }

  public double getXAngle() {
    return tx.getDouble(0);
  }

  public double getYAngle() {
    return ty.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public void log() {
    SmartDashboard.putNumber("Distance", estimateDistanceAlt());
  }
}
