/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import frc.robot.subsystems.BaseSubsystem;

/**
 * Add your docs here.
 */
public class CommandDriveDistanceBuilder {
    private final BaseSubsystem m_baseSubsystem;

    public CommandDriveDistanceBuilder(BaseSubsystem baseSubsystem) {
        m_baseSubsystem = baseSubsystem;
    }

    public CommandDriveDistance build(double x) {
        return new CommandDriveDistance(m_baseSubsystem, x);
    }
}
