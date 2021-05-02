// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.utils.Vision;

public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private final Vision vision = new Vision();

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        var result = vision.camera.getLatestResult();

        if (result.hasTargets()) {
            if (result.getBestTarget().getYaw() <= AutoAimConstants.TOLERANCE) {
                robotContainer
                        .getShuffleBoardHelper()
                        .getIsCameraCentredWidget()
                        .getEntry()
                        .setBoolean(true);
            }
        } else {
            robotContainer
                    .getShuffleBoardHelper()
                    .getIsCameraCentredWidget()
                    .getEntry()
                    .setBoolean(false);
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        var result = vision.camera.getLatestResult();

        if (result.hasTargets()) {
            if (result.getBestTarget().getYaw() <= AutoAimConstants.TOLERANCE) {
                robotContainer
                        .getShuffleBoardHelper()
                        .getIsCameraCentredWidget()
                        .getEntry()
                        .setBoolean(true);
            }
        } else {
            robotContainer
                    .getShuffleBoardHelper()
                    .getIsCameraCentredWidget()
                    .getEntry()
                    .setBoolean(false);
        }
    }
}
