/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseSubsystem extends SubsystemBase {
  // protected final AHRS ahrs;
  protected final DifferentialDrive diffDrive;
  protected final Rev2mDistanceSensor distanceSensor;
  public DriveStyle drive;
  private SendableChooser<DriveStyle> driveChooser;
  public BaseSubsystem(SpeedController left, SpeedController right) {
    // ahrs = new AHRS();
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kLongRange);
    diffDrive = new DifferentialDrive(left, right);
    drive = DriveStyle.SPLIT_ARCADE;
    driveChooser =  new SendableChooser<>();
    driveChooser.addOption("Single Stick Arcade", DriveStyle.SINGLE_ARCADE);
    driveChooser.setDefaultOption("Dual Stick Arcade", DriveStyle.SPLIT_ARCADE);
    driveChooser.addOption("Tank Drive", DriveStyle.TANK_DRIVE);
    driveChooser.addOption("Trigger Arcade", DriveStyle.TRIGGER_ARCADE);
    SmartDashboard.putData("Drive Chooser", driveChooser);
  }

  public boolean isDistanceValid() {
    return distanceSensor.isRangeValid();
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed * inputMultiplier, rotation);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left * inputMultiplier, right * inputMultiplier);
  }

  public void toggleDriveStyle() {
    drive = drive.next();
  }

  public DriveStyle getDriveStyle() {
    Sendable chooser = SmartDashboard.getData("Drive Chooser");
    if(chooser instanceof SendableChooser) {
      Object drive = ((SendableChooser) chooser).getSelected();
      if(drive instanceof DriveStyle) {
        return (DriveStyle) drive;
      } else {
        return null;
      }
    } else {
      return null;
    }
  }

  public void setDriveStyle(DriveStyle driveStyle) {
    this.drive = driveStyle;
  }

  // public double getAngle() {
    // return ahrs.getYaw();
  // }

  // public void resetAngle() {
  //   ahrs.reset();
  // }

  public abstract void toggleShift();

  public abstract void resetEncoder();

  public abstract void driveToTarget(double target);

  public abstract double getDistanceLeft();

  public abstract double getDistanceRight();

  public abstract double getVelocityLeft();

  public abstract double getVelocityRight();

  public double getDistance() {
    return distanceSensor.getRange() / 12.0;
  }

  private double inputMultiplier = 1;

  public void toggleInputMultiplier() {
    if(inputMultiplier == 1) {
      inputMultiplier = 0.5;
    } else {
      inputMultiplier = 1;
    }
  }

  @Override
  public void periodic() {
    log();
  }

  private void log() {
    SmartDashboard.putNumber("Base - Encoder", this.getDistanceLeft());
    SmartDashboard.putNumber("Base - Velocity", this.getVelocityLeft());

    //SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    //SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    //SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    //SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    //SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
    //SmartDashboard.putNumber("IMU_accX", ahrs.getWorldLinearAccelX());
    //SmartDashboard.putNumber("IMU_accY", ahrs.getWorldLinearAccelY());
    //SmartDashboard.putNumber("IMU_accZ", ahrs.getWorldLinearAccelZ());
    //SmartDashboard.putNumber("IMU_angle", ahrs.getAngle());
  }

  public enum DriveStyle {
    SPLIT_ARCADE,
    TANK_DRIVE,
    TRIGGER_ARCADE,
    SINGLE_ARCADE {
      @Override
      public DriveStyle next() {
        return values()[0];
      }
    };

    public DriveStyle next() {
      return values()[ordinal() + 1];
    }
  }
}
