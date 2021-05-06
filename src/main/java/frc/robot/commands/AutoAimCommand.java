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

    private final DriveSubsystem driveSubsystem;
    private final RobotContainer robotContainer;
    private final Vision vision;
    private DoubleSupplier forward;
    private double rotationSpeed;

    // !characterize the robot for these values
    private final PIDController controller =
            new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);

    /**
     * Rotates the robot to the a target if there is one. This command only rotates; you are free to
     * translate the robot.
     *
     * @param driveSubsystem a driveSubsystem instance
     * @param vision a vision instance
     * @param robotContainer a robotContainer instance
     * @param forward the {@code DoubleSupplier} from the driver controller's translation component.
     */
    public AutoAimCommand(
            DriveSubsystem driveSubsystem,
            Vision vision,
            RobotContainer robotContainer,
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
        var result = vision.camera.getLatestResult();

        if (result.hasTargets()
                && (Math.abs(result.getBestTarget().getYaw()) >= AutoAimConstants.TOLERANCE)) {
            rotationSpeed =
                    -controller.calculate(result.getBestTarget().getYaw(), 0)
                            + (Math.copySign(1, result.getBestTarget().getYaw())
                                    * AutoAimConstants.FFW);
        } else {
            rotationSpeed = 0;
        }
        driveSubsystem.arcadeDrive(forward.getAsDouble(), rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        if (controller.atSetpoint()) {
            robotContainer.getRumble().startRumble(0.5);
            return true;
        } else {
            return false;
        }
    }
}
