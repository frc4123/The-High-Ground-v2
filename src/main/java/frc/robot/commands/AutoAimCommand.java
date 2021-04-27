// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveSubsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;

import java.util.function.DoubleSupplier;

public class AutoAimCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DoubleSupplier forward;
    private double rotationSpeed = 0;

    // !characterize the robot for these values
    private static PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
    PhotonPipelineResult result;

    private static PIDController controller =
            new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);

    public AutoAimCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward) {
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        controller.setTolerance(0.01);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        result = camera.getLatestResult();
        if (result.hasTargets() && (Math.abs(result.getBestTarget().getYaw()) >= 0.07)) {
            rotationSpeed =
                    -controller.calculate(result.getBestTarget().getYaw(), 0)
                            + ((result.getBestTarget().getYaw()
                                            / Math.abs(result.getBestTarget().getYaw()))
                                    * AutoAimConstants.FFW);
        } else {
            rotationSpeed = 0;
        }
        driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(result.getBestTarget().getYaw()) <= 0.07) {
            return true;
        } else {
            return false;
        }
    }
}
