// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;

    /** Creates a new ElevatorDownCommand. */
    public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorVelo(.25);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorVelo(0);
    }
}
