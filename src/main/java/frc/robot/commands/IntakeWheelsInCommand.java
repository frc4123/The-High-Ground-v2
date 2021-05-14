// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeWheelsSubsystem;

public class IntakeWheelsInCommand extends CommandBase {
    IntakeWheelsSubsystem intakeWheelsSubsystem;

    /** Creates a new IntakeWheelsInCommand. */
    public IntakeWheelsInCommand(IntakeWheelsSubsystem intakeWheelsSubsystem) {
        this.intakeWheelsSubsystem = intakeWheelsSubsystem;
        addRequirements(intakeWheelsSubsystem);
    }

    @Override
    public void execute() {
        intakeWheelsSubsystem.setIntakeWheelsVelo(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        intakeWheelsSubsystem.setIntakeWheelsVelo(0);
    }
}
