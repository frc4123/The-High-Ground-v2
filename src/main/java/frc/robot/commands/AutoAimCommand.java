// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;

public class AutoAimCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  DoubleSupplier forward;

  double rotationSpeed = 0;

  // !characterize the robot for these values
  PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
  PIDController controller =
      new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);

  public AutoAimCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward) {
    this.driveSubsystem = driveSubsystem;
    this.forward = forward;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      System.out.println("Has Targets");
      rotationSpeed = -controller.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      rotationSpeed = 0;
    }
    driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
  }

  // @Override
  // public boolean isFinished() {
  //   if (Math.abs(result.getBestTarget().getYaw()) <= 0.5) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }
}
