// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import org.photonvision.PhotonCamera;

public class Vision {

  public final PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
  // !characterize the robot for these values
  public final PIDController controller =
      new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);
}
