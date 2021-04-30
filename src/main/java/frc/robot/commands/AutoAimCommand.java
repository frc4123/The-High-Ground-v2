// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Vision;

import java.util.function.DoubleSupplier;

public class AutoAimCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private RobotContainer robotContainer;
    private DoubleSupplier forward;
    private double rotationSpeed;

    // !characterize the robot for these values
    // private static PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);

    private static PIDController controller =
            new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);

    /**
     * Rotates the robot to the a target if there is one. This command only rotates; you are free to
     * translate the robot.
     *
     * @param driveSubsystem the driveSubsystem
     * @param robotContainer the robotContainer
     * @param forward the {@code DoubleSupplier} from the driver controller's translation component.
     */
    public AutoAimCommand(
            DriveSubsystem driveSubsystem, RobotContainer robotContainer, DoubleSupplier forward) {
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.robotContainer = robotContainer;
        rotationSpeed = 0;
        controller.setTolerance(AutoAimConstants.TOLERANCE);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Vision.result = Vision.camera.getLatestResult();
        if (Vision.result.hasTargets()
                && (Math.abs(Vision.result.getBestTarget().getYaw())
                        >= AutoAimConstants.TOLERANCE)) {
            rotationSpeed =
                    -controller.calculate(Vision.result.getBestTarget().getYaw(), 0)
                            + ((Vision.result.getBestTarget().getYaw()
                                            / Math.abs(Vision.result.getBestTarget().getYaw()))
                                    * AutoAimConstants.FFW);
        } else {
            rotationSpeed = 0;
        }
        driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
    }

    @Override
    public boolean isFinished() {

        if (controller.atSetpoint()) {
            // TODO not running for some reason
            new RumbleCommand(robotContainer.getDriverController(), 0.5, 0.5).schedule();
            return true;
        } else {
            return false;
        }
    }
}
