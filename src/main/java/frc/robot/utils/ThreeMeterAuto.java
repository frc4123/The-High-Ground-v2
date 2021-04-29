// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystem;

public class ThreeMeterAuto {

    private final Trajectory path;
    private final Command command;

    public ThreeMeterAuto(DriveSubsystem driveSubsystem) {
        path = driveSubsystem.pathList.get(0);

        command =
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> driveSubsystem.resetPose(path.getInitialPose()),
                                driveSubsystem),
                        driveSubsystem.ramsete(path),
                        new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem));
    }

    public Command getCommand() {
        System.out.println("Three meters");
        return command;
    }
}
