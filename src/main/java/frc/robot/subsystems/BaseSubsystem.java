/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import static frc.robot.Constants.*;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseSubsystem extends SubsystemBase {
  protected final AHRS ahrs;
  protected final DifferentialDrive diffDrive;
  protected final Rev2mDistanceSensor distanceSensor;
  public DriveStyle drive;
  private final SendableChooser<DriveStyle> driveChooser;
  private final DifferentialDriveOdometry odometry;
  private final SpeedController controllerLeft, controllerRight;

  public boolean recording = false;
  public List<Float> steer = new ArrayList<>();
  public List<Float> throttle = new ArrayList<>();

  public BaseSubsystem(SpeedController left, SpeedController right) {
    controllerLeft = left;
    controllerRight = right;
    ahrs = new AHRS();

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kLongRange);
    diffDrive = new DifferentialDrive(left, right);
    drive = DriveStyle.SPLIT_ARCADE;
    driveChooser =  new SendableChooser<>();
    driveChooser.addOption("Single Stick Arcade", DriveStyle.SINGLE_ARCADE);
    driveChooser.setDefaultOption("Dual Stick Arcade", DriveStyle.SPLIT_ARCADE);
    driveChooser.addOption("Tank Drive", DriveStyle.TANK_DRIVE);
    driveChooser.addOption("Trigger Arcade", DriveStyle.TRIGGER_ARCADE);
    //ahrs.reset();
    odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    SmartDashboard.putData("Drive Chooser", driveChooser);
    //resetOdometry(new Pose2d(1.32, -2.411, getRotation2d()));
  }

  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public boolean isDistanceValid() {
    return distanceSensor.isRangeValid();
  }

  public void tankDriveVolts(double l, double r) {
    controllerLeft.setVoltage(-l);
    controllerRight.setVoltage(r);
    diffDrive.feed();
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation * inputMultiplier);
    
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(right * inputMultiplier, left * inputMultiplier);
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

  public double getAngle() {
    return -ahrs.getAngle();
  }

  public void resetAngle() {
    ahrs.reset();
  }

  public abstract void toggleShift();
  public abstract void setShift(boolean x);

  public abstract void resetEncoder();

  public abstract void driveToTarget(double target);

  public abstract double getDistanceLeft();

  public abstract double getDistanceRight();

  public abstract double getVelocityLeft();

  public abstract double getVelocityRight();

  public Command getTrajectoryCommand(String file) {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file);
    try {
      TrajectoryConfig config = new TrajectoryConfig(1.5, 2);
      var feedForward = new SimpleMotorFeedforward(kBaseS, kBaseV, kBaseA);
      var vConstraint = new DifferentialDriveVoltageConstraint(feedForward, kBaseKinematics, 10);
      config.setKinematics(kBaseKinematics);
      config.addConstraint(vConstraint);
      //Bounce
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(1.227, -2.33, new Rotation2d(0)),
       List.of(
         new Translation2d(2.153, -2.045)
       ),
        new Pose2d(2.283, -0.744, new Rotation2d(1.57)), config);
      config.setReversed(true);
      ControlVectorList list = new ControlVectorList();
      list.add(new ControlVector(new double[]{2.283, 0.695, 0.0}, new double[]{-0.744, -.868, 0.0}));
      list.add(new ControlVector(new double[]{3.856, 2.567, 0.0}, new double[]{-3.818, 0, 0.0}));
      list.add(new ControlVector(new double[]{4.695, 0, 0.0}, new double[]{-0.757, 0.66, 0.0}));
      Trajectory traj2 = TrajectoryGenerator.generateTrajectory(new Pose2d(2.283, -0.744, new Rotation2d(1.7)),
      List.of(
       new Translation2d(3.501, -3.1),
       new Translation2d(4.6, -3.568)
      ),
       new Pose2d(4.821, -0.757, new Rotation2d(-1.57)), config);
      config.setReversed(false);
      Trajectory traj3 = TrajectoryGenerator.generateTrajectory(new Pose2d(4.821, -0.757, new Rotation2d(-1.7)),
       List.of(
        new Translation2d(5.046, -3),
        new Translation2d(6.311, -3.468)
       ),
        new Pose2d(6.961, -.5, new Rotation2d(1.57)), config);
      config.setReversed(true);
      Trajectory traj4 = TrajectoryGenerator.generateTrajectory(new Pose2d(6.961, -.5, new Rotation2d(1.7)),
       List.of(
       ),
        new Pose2d(8.04, -1.4, new Rotation2d(-3.1419)), config);
      
      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(kRamseteB, kRamseteZ),
        feedForward, 
        kBaseKinematics,
        this::getWheelSpeeds,
        new PIDController(kBasePV, 0, 0),
        new PIDController(kBasePV, 0, 0),
        this::tankDriveVolts, this);

        RamseteCommand ramseteCommand2 = new RamseteCommand(
          traj2,
          this::getPose,
          new RamseteController(kRamseteB, kRamseteZ),
          feedForward, 
          kBaseKinematics,
          this::getWheelSpeeds,
          new PIDController(kBasePV, 0, 0),
          new PIDController(kBasePV, 0, 0),
          this::tankDriveVolts, this);
          RamseteCommand ramseteCommand3 = new RamseteCommand(
            traj3,
            this::getPose,
            new RamseteController(kRamseteB, kRamseteZ),
            feedForward, 
            kBaseKinematics,
            this::getWheelSpeeds,
            new PIDController(kBasePV, 0, 0),
            new PIDController(kBasePV, 0, 0),
            this::tankDriveVolts, this);
            RamseteCommand ramseteCommand4 = new RamseteCommand(
              traj4,
              this::getPose,
              new RamseteController(kRamseteB, kRamseteZ),
              feedForward, 
              kBaseKinematics,
              this::getWheelSpeeds,
              new PIDController(kBasePV, 0, 0),
              new PIDController(kBasePV, 0, 0),
              this::tankDriveVolts, this);
      Command command = new InstantCommand(() -> {
        setShift(true);
        resetEncoder();
        resetOdometry(new Pose2d(new Translation2d(1.227, -2.33), getRotation2d()));
      }, this);
      return command.andThen(ramseteCommand).andThen(ramseteCommand2.andThen(ramseteCommand3).andThen(ramseteCommand4).andThen(() -> tankDriveVolts(0, 0)));
    } catch(Exception e) {
      return null;
    }
  }

  public Command getBarrelCommand2() {
    try {
      TrajectoryConfig config = new TrajectoryConfig(1.5, 2);
      var feedForward = new SimpleMotorFeedforward(kBaseS, kBaseV, kBaseA);
      var vConstraint = new DifferentialDriveVoltageConstraint(feedForward, kBaseKinematics, 10);
      config.setKinematics(kBaseKinematics);
      config.addConstraint(vConstraint);
      ControlVectorList list = new ControlVectorList();
      list.add(new ControlVector(new double[]{1.123, 0.695, 0.0}, new double[]{-2.283, -0.037, 0.0}));
      list.add(new ControlVector(new double[]{4.966, 0.61, 0.0}, new double[]{-2.591, -0.965, 0.0}));
      list.add(new ControlVector(new double[]{4.531, -0.683, 0.0}, new double[]{-3.906, -0.463, 0.0}));
      list.add(new ControlVector(new double[]{3.34, -.9791, 0.0}, new double[]{-3.789, 0.489, 0}));
      list.add(new ControlVector(new double[]{3.584, 1.189, 0}, new double[]{-2.274, 0.598, 0}));
      list.add(new ControlVector(new double[]{7.664, -0.577, 0}, new double[]{-1.969, 1.67, 0}));
      list.add(new ControlVector(new double[]{6.963, -1.381, 0}, new double[]{-0.65, 0.035, 0}));
      list.add(new ControlVector(new double[]{4.853, 0.203, 0}, new double[]{-1.3, -1.462, 0}));
      list.add(new ControlVector(new double[]{7.772, 1.67, 0}, new double[]{-3.848, 0.04, 0}));
      list.add(new ControlVector(new double[]{8.674, -2.144, 0}, new double[]{-1.7, 0.14, 0}));
      list.add(new ControlVector(new double[]{1.163, -0.515, 0}, new double[]{-2.155, 0.034, 0}));
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(list,
         config);
      
      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(2.0, 0.8),
        feedForward, 
        kBaseKinematics,
        this::getWheelSpeeds,
        new PIDController(kBasePV, 0, 0.0),
        new PIDController(kBasePV, 0, 0.0),
        this::tankDriveVolts, this);
      Command command = new InstantCommand(() -> {
        setShift(!usingHighGear);
        resetEncoder();
        resetOdometry(new Pose2d(new Translation2d(1.123, -2.283), getRotation2d()));
      }, this);
      return command.andThen(ramseteCommand).andThen(() -> tankDriveVolts(0, 0));
    } catch(Exception e) {
      return null;
    }
  }

  public Command getBarrelCommand() {
    try {
      TrajectoryConfig config = new TrajectoryConfig(0.5, 1);
      var feedForward = new SimpleMotorFeedforward(kBaseS, kBaseV, kBaseA);
      var vConstraint = new DifferentialDriveVoltageConstraint(feedForward, kBaseKinematics, 10);
      config.setKinematics(kBaseKinematics);
      config.addConstraint(vConstraint);
      //Bounce
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(1.123, -2.283, new Rotation2d(0)),
       List.of(
        new Translation2d(5.3, -2.9),
        new Translation2d(4.53, -3.9),
        new Translation2d(3.24, -4.051),
        new Translation2d(2.6, -2.574),
        new Translation2d(7.912, -1.554),
        new Translation2d(6.10, -.8),
        new Translation2d(4.915, -0.953),
        new Translation2d(7.772, -3.848),
        new Translation2d(8.623, -2.172)
       ),
       new Pose2d(1.123, -2.283, new Rotation2d(3.1415)),
         config);
      
      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(1.0, 1.0),
        feedForward, 
        kBaseKinematics,
        this::getWheelSpeeds,
        new PIDController(kBasePV, 0, 0),
        new PIDController(kBasePV, 0, 0),
        this::tankDriveVolts, this);
      Command command = new InstantCommand(() -> {
        setShift(!usingHighGear);
        resetEncoder();
        resetOdometry(new Pose2d(new Translation2d(1.123, -2.283), getRotation2d()));
      }, this);
      return command.andThen(ramseteCommand).andThen(() -> tankDriveVolts(0, 0));
    } catch(Exception e) {
      return null;
    }
  }

  public Command getShallomCommand() {
    try {
      TrajectoryConfig config = new TrajectoryConfig(1, 1);
      var feedForward = new SimpleMotorFeedforward(kBaseS, kBaseV, kBaseA);
      var vConstraint = new DifferentialDriveVoltageConstraint(feedForward, kBaseKinematics, 10);
      config.setKinematics(kBaseKinematics);
      config.addConstraint(vConstraint);
      //Bounce
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
       List.of(
        new Pose2d(1.051,-3.851, new Rotation2d(0.0147048223778514)),
new Pose2d(3.006,-2.234, new Rotation2d(0.264154015442657)),
new Pose2d(6.458,-2.161, new Rotation2d(-1.44945068531984)),
new Pose2d(7.921,-3.924, new Rotation2d(0.366584495947189)),
new Pose2d(8.664,-2.934, new Rotation2d(1.3703292540065)),
new Pose2d(7.569,-1.766, new Rotation2d(-2.82118198226806)),
new Pose2d(6.118,-3.75, new Rotation2d(-2.71998267982143)),
new Pose2d(3.09,-3.648, new Rotation2d(2.47914696708714)),
new Pose2d(1.282,-1.852, new Rotation2d(-0.0322468824352539))
       ),
         config);
      
      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(2.0, 0.8),
        feedForward, 
        kBaseKinematics,
        this::getWheelSpeeds,
        new PIDController(kBasePV, 0, 0),
        new PIDController(kBasePV, 0, 0),
        this::tankDriveVolts, this);
      Command command = new InstantCommand(() -> {
        setShift(true);
        resetEncoder();
        resetOdometry(new Pose2d(new Translation2d(1.051,-3.851), getRotation2d()));
      }, this);
      return command.andThen(ramseteCommand).andThen(() -> tankDriveVolts(0, 0));
    } catch(Exception e) {
      return null;
    }
  }

  public double getDistance() {
    return distanceSensor.getRange() / 12.0;
  }

  private double inputMultiplier = 1;

  public void toggleInputMultiplier() {
    if(inputMultiplier == 1) {
      inputMultiplier = 0.8;
    } else {
      inputMultiplier = 1;
    }
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getDistanceLeft(), getDistanceRight());
    log();
  }

  private void log() {
    SmartDashboard.putBoolean("Base - Precision mode", this.inputMultiplier != 1.0);
    SmartDashboard.putNumber("Base - PosX", getPose().getTranslation().getX());
    SmartDashboard.putNumber("Base - PosY", getPose().getTranslation().getY());
    SmartDashboard.putNumber("Base dist", getDistanceLeft());
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

  public Command getRedB(IntakeSubsystem intake) {
    try {
      TrajectoryConfig config = new TrajectoryConfig(1.25, 2);
      var feedForward = new SimpleMotorFeedforward(kBaseS, kBaseV, kBaseA);
      var vConstraint = new DifferentialDriveVoltageConstraint(feedForward, kBaseKinematics, 10);
      config.setKinematics(kBaseKinematics);
      config.addConstraint(vConstraint);
      config.setReversed(true);
      Trajectory traj2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0.775, -1.517, new Rotation2d(-3.141592)),
      List.of(
      ),
       new Pose2d(2.295, 0, new Rotation2d(-1.5707)), config);
      config.setReversed(false);
      ControlVectorList list = new ControlVectorList();
      list.add(new ControlVector(new double[]{2.295, 0, 0.0}, new double[]{-0.096, -1.479, 0.0}));
      list.add(new ControlVector(new double[]{3.623, 2.646, 0.0}, new double[]{-2.653, 0, 0.0}));
      list.add(new ControlVector(new double[]{4.717, 1.566, 0.0}, new double[]{-1.029, -0, 0.0}));
      list.add(new ControlVector(new double[]{8.758, 2.216, 0.0}, new double[]{-1.129, -0, 0.0}));
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(list,
         config);
      
      RamseteCommand ramseteCommand = new RamseteCommand(
        traj2,
        this::getPose,
        new RamseteController(2.0, 0.8),
        feedForward, 
        kBaseKinematics,
        this::getWheelSpeeds,
        new PIDController(kBasePV, 0, 0.0),
        new PIDController(kBasePV, 0, 0.0),
        this::tankDriveVolts, this);
        RamseteCommand ramseteCommand2 = new RamseteCommand(
          trajectory,
          this::getPose,
          new RamseteController(2.0, 0.8),
          feedForward, 
          kBaseKinematics,
          this::getWheelSpeeds,
          new PIDController(kBasePV, 0, 0.0),
          new PIDController(kBasePV, 0, 0.0),
          this::tankDriveVolts, this);
      Command command = new InstantCommand(() -> {
        setShift(!usingHighGear);
        resetEncoder();
        resetOdometry(new Pose2d(new Translation2d(0.775, -1.517), new Rotation2d(-3.141592)));
      }, this);
      return command.andThen(ramseteCommand).andThen(() -> {
        tankDriveVolts(0, 0);
        intake.toggleIntake();
        intake.runIntake(1);
      }).andThen(ramseteCommand2)
      .andThen(() -> {
        tankDriveVolts(0, 0);
        intake.toggleIntake();
        intake.runIntake(0);
      });
    } catch(Exception e) {
      return null;
    }
  }
}
