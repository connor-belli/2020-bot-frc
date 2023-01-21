/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public final static int kMotorBaseL1 = 1;
    public final static int kMotorBaseL2 = 2;
    public final static int kMotorBaseR1 = 3;
    public final static int kMotorBaseR2 = 4;

    public final static int kMotorIntake = 5;
    public final static int kMotorHopper = 6;

    public final static int kMotorIndex = 7;
    public final static int kMotorTurret = 8;
    public final static int kMotorLeftShooter = 9;
    public final static int kMotorRightShooter = 10;

    public final static int kMotorClimb = 11;

    public final static int kMotorControlPanel = 13;
    public final static int kMotorDump = 14;

    public final static int kMotorHood = 12;

    public final static int kTurretLimitLeft = 1;
    public final static int kTurretLimitRight = 2;
    public final static int kShooterLimitTop = 3;
    public final static int kShooterLimitBottom = 4;
    public final static int kClimberLimit = 5;

    public final static double kTurretLeftLimitAngle = -70;
    public final static double kTurretRightLimitAngle = 70;
    public final static double kShooterBottomLimitAngle = 59;
    public final static double kShooterTopLimitAngle = 10;

    public final static Color kBlueTarget = ColorMatch.makeColor(.18, .45, .36);
    public final static Color kGreenTarget = ColorMatch.makeColor(.21, .51, .26);
    public final static Color kRedTarget = ColorMatch.makeColor(.31, .44, .23);
    public final static Color kYellowTarget = ColorMatch.makeColor(.28, .51, .19);

    public final static double kTargetRatio = 39.25 / 30.0;
    public final static double kTargetSlope = 8.928;

    public final static double kProjectileRadius = 0.0889;
    public final static double kProjectileCoeff = 0.8;
    public final static double kProjectileMass = 0.1417;
    public final static double kGravity = 9.81;
    public final static double kAirDensity = 1.24;
    public final static double kProjectileArea = kProjectileRadius * kProjectileRadius * Math.PI;
    public final static double kProjectileConstant = kProjectileCoeff * kAirDensity * kProjectileArea /
            (2 * kGravity * kProjectileMass);
    public final static double kBaseP = 0.006;
    public final static double kBaseI = 0.00001;
    public final static double kBaseD = 0.000015;
    public final static double kBaseF = 0.0;

    public final static double kTurretP = 3.00;
    public final static double kTurretI = 0.0;
    public final static double kTurretD = 0.0;
    public final static double kTurretF = 0.0;

    public final static double kShooterP = 0.0;
    public final static double kShooterI = 0.0;
    public final static double kShooterD = 0.0;
    public final static double kShooterF = 0.042;

    public final static double kShooterHoodP = 4.0;
    public final static double kShooterHoodI = 0.0;
    public final static double kShooterHoodD = 0.0;
    public final static double kShooterHoodF = 0.0;

    // Shooter values
    public final static double kShooterMin = 0.1;
    public final static double kShooterLow = 0.25;
    public final static double kShooterMed = 0.5;
    public final static double kShooterHigh = 1.0;

    public final static double kShooterTopSpeed = 6000;//some number to be determined
    public final static double kShooterTopVelocity = 34;
    public final static double kQuadInc = 0.04;
    public final static double kTargetHeight = 89.5;
    public final static double kLimeLightHeight = 21.15;
    public final static double kLimeLightAngle = 30;

    public final static double kHoodA = -0.025;
    public final static double kHoodB = 26;
    public final static double kPressureA = 120/2200;
    public final static double kPressureB = 0;
    public final static boolean usingHighGear = false;
    public final static double kBaseS = usingHighGear?1.5:0.5;
    public final static double kBaseV = usingHighGear?.5:3.34;
    public final static double kBaseA = usingHighGear?0.213:0.282;

    public final static double kBasePV = usingHighGear?1.0:1.71;//0.109;
    public final static double kBaseW = 0.62614;

    public final static double kBaseMaxAcc = 1;
    public final static double kBaseMaxSpeed = 0.2;

    public static final double kRamseteB = 2;
    public static final double kRamseteZ = 0.7;
    public static final DifferentialDriveKinematics kBaseKinematics = new DifferentialDriveKinematics(kBaseW);

    public static final double kPolyA = 0;
    public static final double kPolyB = 0;
    public static final double kPolyC = 0;
}
