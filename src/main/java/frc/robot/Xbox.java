/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class Xbox {
  public XboxController controller;

  public Xbox(int id) {
    controller = new XboxController(id);
  }

  public double getRightT() {
    return Util.clampAbs(0.1, controller.getTriggerAxis(Hand.kRight));
  }

  public double getLeftT() {
    return Util.clampAbs(0.1, controller.getTriggerAxis(Hand.kLeft));
  }

  public double getRightY() {
    return Util.clampAbs(0.03, controller.getY(Hand.kRight));
  }

  public double getRightX() {
    return Util.clampAbs(0.1, controller.getX(Hand.kRight));
  }

  public double getLeftY() {
    return Util.clampAbs(0.03, controller.getY(Hand.kLeft));
  }

  public double getLeftX() {
    return Util.clampAbs(0.03, controller.getX(Hand.kLeft));
  }
}
