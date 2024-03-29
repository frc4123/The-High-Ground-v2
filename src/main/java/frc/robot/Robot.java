// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.utils.Vision;

import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private static PhotonPipelineResult result;
    private Vision vision;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        vision = new Vision();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // This routine updates a boolean box based on if the robot is lined up with the
        // target
        result = vision.camera.getLatestResult();

        if (result != null && result.hasTargets()) {
            // distance update
            robotContainer.getShuffleBoardHelper().getDistanceToTargetWidget().getEntry()
                    .setDouble(PhotonUtils.calculateDistanceToTargetMeters(AutoAimConstants.CAMERA_HEIGHT_METERS,
                            AutoAimConstants.TARGET_HEIGHT_METERS, AutoAimConstants.CAMERA_PITCH_RADIANS,
                            Math.toRadians(Robot.getResult().getBestTarget().getPitch())));
            // boolean update
            if (Math.abs(result.getBestTarget().getYaw()) <= AutoAimConstants.TOLERANCE) {
                robotContainer.getShuffleBoardHelper().getIsCameraCentredWidget().getEntry().setBoolean(true);
            } else {
                robotContainer.getShuffleBoardHelper().getIsCameraCentredWidget().getEntry().setBoolean(false);
            }
        } else {
            robotContainer.getShuffleBoardHelper().getIsCameraCentredWidget().getEntry().setBoolean(false);
        }

        // Rumble still doesn't work
        robotContainer.rumble.periodic();
    }

    public static PhotonPipelineResult getResult() {
        return result;
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

}
