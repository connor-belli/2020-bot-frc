/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BaseSubsystem;

public class AutoRobotCharacterization extends CommandBase {
  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];
  /**
   * Creates a new AutoRobotCharacterization.
   */
  private final BaseSubsystem m_baseSubsystem;
  public AutoRobotCharacterization(BaseSubsystem baseSubsystem) {
    m_baseSubsystem = baseSubsystem;
    addRequirements(baseSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    priorAutospeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = m_baseSubsystem.getDistanceLeft();
    double leftRate = m_baseSubsystem.getVelocityLeft();

    double rightPosition = m_baseSubsystem.getDistanceRight();
    double rightRate = m_baseSubsystem.getVelocityRight();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    m_baseSubsystem.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed
    );

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = Math.toRadians(m_baseSubsystem.getAngle());

    telemetryEntry.setNumberArray(numberArray);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
