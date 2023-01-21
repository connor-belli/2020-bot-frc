/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.autos.AutoEnemyTrenchRun;
import frc.robot.commands.autos.AutoShootInitLine;
import frc.robot.commands.autos.AutoTrenchRunAlliance;
import frc.robot.commands.base.*;
import frc.robot.commands.hood.CommandAutoAimHood;
import frc.robot.commands.hood.CommandHoodToPos;
import frc.robot.commands.hood.CommandRunShooterHood;
import frc.robot.commands.shooter.CommandAlignToTarget;
import frc.robot.commands.shooter.CommandToggleShooter;
import frc.robot.commands.turret.CommandAimTurret;
import frc.robot.commands.turret.CommandRunTurret;
import frc.robot.subsystems.*;
import frc.robot.subsystems.comp.CompBaseSubsystem;
import frc.robot.subsystems.comp.CompIntakeSubsystem;
import frc.robot.subsystems.practice.PracticeBaseSubsystem;
import frc.robot.subsystems.practice.PracticeIntakeSubsystem;

import static frc.robot.Constants.kShooterBottomLimitAngle;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final BaseSubsystem m_baseSubsystem;
  //private final ClimberSubsystem m_climberSubsystem;
  //private final ControlPanelSubsystem m_controlPanelSubsystem;
  private final IndexWheelSubsystem m_indexWheelSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final ShooterHoodSubsystem m_shooterHoodSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterTurretSubsystem m_shooterTurretSubsystem;

  private final Xbox m_driver = new Xbox(0);
  private final Xbox m_operator = new Xbox(1);

  private final JoystickButton m_driver_a = new JoystickButton(m_driver.controller, XboxController.Button.kA.value);
  private final JoystickButton m_driver_b = new JoystickButton(m_driver.controller, XboxController.Button.kB.value);
  private final JoystickButton m_driver_x = new JoystickButton(m_driver.controller, XboxController.Button.kX.value);
  private final JoystickButton m_driver_y = new JoystickButton(m_driver.controller, XboxController.Button.kY.value);
  private final JoystickButton m_driver_rb = new JoystickButton(m_driver.controller, XboxController.Button.kBumperRight.value);
  private final JoystickButton m_driver_lb = new JoystickButton(m_driver.controller, XboxController.Button.kBumperLeft.value);
  private final JoystickButton m_driver_start = new JoystickButton(m_driver.controller, XboxController.Button.kStart.value);
  private final POVButton m_driver_up = new POVButton(m_driver.controller, 0);
  private final POVButton m_driver_down = new POVButton(m_driver.controller, 180);

  private final JoystickButton m_operator_a = new JoystickButton(m_operator.controller, XboxController.Button.kA.value);
  private final JoystickButton m_operator_b = new JoystickButton(m_operator.controller, XboxController.Button.kB.value);
  private final JoystickButton m_operator_x = new JoystickButton(m_operator.controller, XboxController.Button.kX.value);
  private final JoystickButton m_operator_y = new JoystickButton(m_operator.controller, XboxController.Button.kY.value);
  private final JoystickButton m_operator_lb = new JoystickButton(m_operator.controller, XboxController.Button.kBumperLeft.value);
  private final JoystickButton m_operator_rb = new JoystickButton(m_operator.controller, XboxController.Button.kBumperRight.value);
  private final JoystickButton m_operator_back = new JoystickButton(m_operator.controller, XboxController.Button.kBack.value);
  private final JoystickButton m_operator_start = new JoystickButton(m_operator.controller, XboxController.Button.kStart.value);

  private final POVButton m_operator_up = new POVButton(m_operator.controller, 0);
  private final POVButton m_operator_right = new POVButton(m_operator.controller, 90);
  private final POVButton m_operator_down = new POVButton(m_operator.controller, 180);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  private final boolean isComp = true;
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    if (isComp) {
      m_baseSubsystem = new CompBaseSubsystem();
      m_intakeSubsystem = new CompIntakeSubsystem();
      //m_controlPanelSubsystem = new ControlPanelSubsystem();
      //m_climberSubsystem = null; //new ClimberSubsystem();
      m_shooterHoodSubsystem = new ShooterHoodSubsystem();
      m_limeLightSubsystem = new LimeLightSubsystem();
      m_shooterSubsystem = new ShooterSubsystem();
      m_shooterTurretSubsystem = new ShooterTurretSubsystem();
    } else {
      m_baseSubsystem = new PracticeBaseSubsystem();
      m_intakeSubsystem = new PracticeIntakeSubsystem();
      //m_climberSubsystem = null;
      //m_controlPanelSubsystem = null;
      m_shooterHoodSubsystem = null;
      m_limeLightSubsystem = null;
      m_shooterSubsystem = null;
      m_shooterTurretSubsystem = null;
    }
    m_indexWheelSubsystem = new IndexWheelSubsystem();

    m_baseSubsystem.setDefaultCommand(new CommandRunBase(m_baseSubsystem, m_driver));
    m_indexWheelSubsystem.setDefaultCommand(new CommandRunConveyor(m_indexWheelSubsystem, m_operator, m_shooterSubsystem));
    if (isComp) {
      m_shooterTurretSubsystem.setDefaultCommand(new CommandRunTurret(m_shooterTurretSubsystem, m_operator));
      m_shooterHoodSubsystem.setDefaultCommand(new CommandRunShooterHood(m_shooterHoodSubsystem, m_operator));
      //m_climberSubsystem.setDefaultCommand(new CommandRunClimberMotor(m_climberSubsystem, m_driver));
    }

    // Configure the button bindings
    configureButtonBindings();
    initAutoChooser();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (isComp) {
      m_driver_x.whenPressed(new CommandToggleVision(m_limeLightSubsystem));
      //m_driver_rb.whenPressed(new CommandToggleRecord(m_baseSubsystem));
      //m_driver_y.whenPressed(new CommandPlayRecording(m_baseSubsystem));
      //m_driver_lb.whenPressed(new CommandToggleClimber(m_climberSubsystem));
      //m_driver_rb.whenPressed(new InstantCommand(() -> m_intakeSubsystem.setIntake(false), m_intakeSubsystem));
      //m_driver_a.whileHeld(new CommandAlignToTarget(m_baseSubsystem, m_limeLightSubsystem, m_driver));
      m_driver_start.whenPressed(new CommandTogglePrecision(m_baseSubsystem));


      m_operator_a.whileHeld(new ParallelCommandGroup(new CommandAutoAimHood(m_shooterHoodSubsystem, m_limeLightSubsystem), new CommandAimTurret(m_shooterTurretSubsystem, m_limeLightSubsystem)));
      m_operator_x.whileHeld(new CommandIntakeForward(m_intakeSubsystem));
      m_operator_b.whileHeld(new CommandIntakeReverse(m_intakeSubsystem));
      m_operator_lb.whenPressed(new CommandToggleIntakeNoMotor(m_intakeSubsystem));
      m_operator_rb.whenPressed(new CommandToggleShooter(m_shooterSubsystem));
      m_operator_start.whenPressed(new InstantCommand(() -> System.out.println("(" + m_limeLightSubsystem.getYAngle() + ", " + m_shooterHoodSubsystem.getTargetAngle() + ")")));
      m_operator_up.whenPressed(new CommandHoodToPos(m_shooterHoodSubsystem, 40));
      m_operator_right.whenPressed(new CommandHoodToPos(m_shooterHoodSubsystem, 30));
      m_operator_down.whenPressed(new CommandHoodToPos(m_shooterHoodSubsystem, kShooterBottomLimitAngle));
    } else {
      m_driver_lb.whenPressed(new CommandToggleDriveStyle(m_baseSubsystem));
    }
    m_driver_b.whenPressed(new CommandToggleShift(m_baseSubsystem));
    //m_driver_rb.whenPressed(new CommandToggleIntake(m_intakeSubsystem));
  }

  private void initAutoChooser() {
    autoChooser = new SendableChooser<>();
    if(isComp) {
      autoChooser.addOption("Enemy Trench Run", new AutoEnemyTrenchRun(m_baseSubsystem, m_shooterSubsystem, m_shooterHoodSubsystem, m_shooterTurretSubsystem, m_indexWheelSubsystem, m_limeLightSubsystem, m_intakeSubsystem));
      autoChooser.addOption("Alliance Trench Run", new AutoTrenchRunAlliance(m_baseSubsystem, m_shooterSubsystem, m_shooterHoodSubsystem, m_shooterTurretSubsystem, m_indexWheelSubsystem, m_limeLightSubsystem, m_intakeSubsystem));
      autoChooser.addOption("Shoot Init Line", new AutoShootInitLine(m_baseSubsystem, m_shooterSubsystem, m_shooterHoodSubsystem, m_shooterTurretSubsystem, m_indexWheelSubsystem, m_intakeSubsystem, m_limeLightSubsystem));
      autoChooser.addOption("Drive Forward", new CommandDriveDistance(m_baseSubsystem, 5));
      autoChooser.addOption("Barrel", m_baseSubsystem.getBarrelCommand2());
      autoChooser.addOption("shallots", m_baseSubsystem.getShallomCommand());
      autoChooser.addOption("Bounce", m_baseSubsystem.getTrajectoryCommand("bounce.wpilib.json"));
      autoChooser.addOption("RedB", m_baseSubsystem.getRedB(m_intakeSubsystem));
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *2
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Sendable chooser = SmartDashboard.getData("Auto Chooser");
    if(chooser instanceof SendableChooser) {
      Object command = ((SendableChooser) chooser).getSelected();
      if(command instanceof Command) {
        return (Command) command;
      } else {
        return null;
      }
    } else {
      return null;
    }
  }
}
