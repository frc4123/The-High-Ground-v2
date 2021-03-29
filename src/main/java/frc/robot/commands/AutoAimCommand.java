// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Vision;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;

public class AutoAimCommand extends CommandBase {

  PhotonCamera camera;
  PIDController controller;
  DriveSubsystem driveSubsystem;
  DoubleSupplier forward;

  double rotationSpeed = 0;
  // PhotonPipelineResult result;

  public AutoAimCommand(Vision vision, DriveSubsystem driveSubsystem, DoubleSupplier forward) {
    this.camera = vision.camera;
    this.controller = vision.controller;
    this.driveSubsystem = driveSubsystem;
    this.forward = forward;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      rotationSpeed = -controller.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      rotationSpeed = 0;
      System.out.print("You shouldn't see this!");
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
