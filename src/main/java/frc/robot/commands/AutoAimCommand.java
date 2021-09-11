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

import org.photonvision.PhotonPipelineResult;

import java.util.function.DoubleSupplier;

public class AutoAimCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final RobotContainer robotContainer;
    private final Vision vision;
    private PhotonPipelineResult result;
    private DoubleSupplier forward;
    private double rotationSpeed;
    private boolean isCameraWorking;

    // !characterize the robot for these values
    private final PIDController controller = new PIDController(AutoAimConstants.KP, AutoAimConstants.KI,
            AutoAimConstants.KD);

    /**
     * Rotates the robot to the a target if there is one. This command only rotates;
     * you are free to translate the robot.
     *
     * @param driveSubsystem a driveSubsystem instance
     * @param vision         a vision instance
     * @param robotContainer a robotContainer instance
     * @param forward        the {@code DoubleSupplier} from the driver controller's
     *                       translation component.
     */
    public AutoAimCommand(DriveSubsystem driveSubsystem, Vision vision, RobotContainer robotContainer,
            DoubleSupplier forward) {
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.robotContainer = robotContainer;
        this.vision = vision;
        rotationSpeed = 0;
        controller.setTolerance(AutoAimConstants.TOLERANCE);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        result = vision.camera.getLatestResult();

        if (result != null) {
            isCameraWorking = true;

            if (result.hasTargets() && (Math.abs(result.getBestTarget().getYaw()) >= AutoAimConstants.TOLERANCE)) {
                // this works
                // robotContainer.rumble.startRumble(0.5);
                rotationSpeed = -controller.calculate(result.getBestTarget().getYaw(), 0)
                        + (Math.copySign(1, result.getBestTarget().getYaw()) * AutoAimConstants.FFW);
            } else {
                rotationSpeed = 0;
            }
            driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
        } else {
            isCameraWorking = false;
            System.out.println("The camera could not be found.");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // always reaches end, only fully goes through "proper" flow in exception listed
        // on trello
        System.out.println("End");
        if (controller.atSetpoint()) {
            System.out.println("End set-point");
            robotContainer.rumble.startRumble(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO is there somewhere else making this command stop?

        if (isCameraWorking && Math.abs(result.getBestTarget().getYaw()) <= AutoAimConstants.TOLERANCE) {
            System.out.println("finished set-point");
            return true;
        } else {
            return false;
        }

    }
}
