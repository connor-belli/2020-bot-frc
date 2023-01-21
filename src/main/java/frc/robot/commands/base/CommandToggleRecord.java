/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.base;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BaseSubsystem;

public class CommandToggleRecord extends InstantCommand {
  /**
   * Creates a new CommandToggleRecord.
   */
  public CommandToggleRecord(BaseSubsystem m_baseSubsystem) {
    super(() -> {
      m_baseSubsystem.recording = !m_baseSubsystem.recording;
      if(m_baseSubsystem.recording) {
        System.out.println("Started recording");
        m_baseSubsystem.steer = new ArrayList<>();
        m_baseSubsystem.throttle = new ArrayList<>();
      } else {
        System.out.println("Stopped recording");
      }
    }, m_baseSubsystem);
  }
}
