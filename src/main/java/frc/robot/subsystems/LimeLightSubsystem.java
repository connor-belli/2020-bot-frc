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
import frc.robot.ProjectileMath;

import static frc.robot.Constants.*;

public class LimeLightSubsystem extends SubsystemBase {
  private boolean vision = false;
  private NetworkTableEntry tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0");
  private NetworkTableEntry tx1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1");
  private NetworkTableEntry tx2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx2");

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

  public int getNumTargets() {
    return 0;
  }

  public double getSize() {
    return ta.getDouble(0);
  }


  public void useVision(boolean b) {
    if (b) {
      camMode.setNumber(0);
      ledMode.setNumber(3);
    } else {
      camMode.setNumber(1);
      ledMode.setNumber(1);
    }
    SmartDashboard.putBoolean("using vision", b);
    vision = b;
  }

  public boolean usingVision() {
    return SmartDashboard.getBoolean("using vision", false);
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
    return 1 / Math.tan(Math.toRadians(angle + kLimeLightAngle));
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
    if (SmartDashboard.getBoolean("using vision", false)) {
      camMode.setNumber(0);
      ledMode.setNumber(3);
    } else {
      camMode.setNumber(1);
      ledMode.setNumber(1);
    }
    SmartDashboard.putString("Path", getGalacticType().name());

    // try {
    //   double distance = estimateDistanceAlt() * 0.0254;
    //   double height = (kTargetHeight - kLimeLightHeight) * 0.0254;
    //   double[] params = ProjectileMath.getOptimumParams(distance, height);
    //   SmartDashboard.putNumber("Error squared", ProjectileMath.minDist(params[0], params[1], kProjectileConstant, kGravity, distance, height, 0.01));
    //   SmartDashboard.putNumber("Estimated angle", Math.toDegrees(params[0]));
    // } catch(Exception e) {
      
    // }
  }

  public static enum GalacticType {
    BlueA,
    BlueB,
    RedA,
    RedB
  }
  double thresh = 0.1;
  public GalacticType getGalacticType() {
    double tx0 = this.tx0.getDouble(0);
    double tx1 = this.tx1.getDouble(0);
    double tx2 = this.tx2.getDouble(0);
    boolean m0 = Math.abs(tx0) < thresh;
    boolean m1 = Math.abs(tx1) < thresh;
    boolean m2 = Math.abs(tx2) < thresh;
    double avg = (tx0 + tx1 + tx2)/3;
    if(m0 && m1 || m1 && m2 || m2 && m0) {
      //B
      if(avg > 0) {
        return GalacticType.RedB;
      } else {
        return GalacticType.BlueB;
      }
    } else {
      if(avg < 0) {
        return GalacticType.RedA;
      } else {
        return GalacticType.BlueA;
      }
    }
  }
}
