// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShuffleBoardHelper;

public class RobotContainer {

    private final XboxController driverController =
            new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final ShuffleBoardHelper shuffleBoardHelper;

    private final AutoAimCommand autoAimCommand =
            new AutoAimCommand(
                    driveSubsystem, this, () -> -driverController.getY(GenericHID.Hand.kLeft));

    private final StartEndCommand elevatorDownCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorVelocity(-0.25),
                    () -> elevatorSubsystem.setElevatorVelocity(0),
                    elevatorSubsystem);

    private final StartEndCommand elevatorUpCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorVelocity(0.25),
                    () -> elevatorSubsystem.setElevatorVelocity(0),
                    elevatorSubsystem);

    private final StartEndCommand shootCommand =
            new StartEndCommand(
                    () -> shooterSubsystem.setSpeed(0.25),
                    () -> shooterSubsystem.setSpeed(0),
                    shooterSubsystem);

    private void calibrate() {
        System.out.println("Gyro is calibrating...");
        driveSubsystem.calibrateGyro();
    }

    public RobotContainer() {
        calibrate();
        configureButtonBindings();
        shuffleBoardHelper = new ShuffleBoardHelper(driveSubsystem);

        driveSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> {
                            driveSubsystem.arcadeDrive(
                                    -driverController.getY(GenericHID.Hand.kLeft),
                                    driverController.getX(GenericHID.Hand.kRight));
                        },
                        driveSubsystem));
    }

    private void configureButtonBindings() {
        // create buttons
        Button lb = new JoystickButton(driverController, XboxConstants.LB_BUTTON);
        Button rb = new JoystickButton(driverController, XboxConstants.RB_BUTTON);
        Button a = new JoystickButton(driverController, XboxConstants.A_BUTTON);
        // Button b = new JoystickButton(driverController, XboxConstants.B_BUTTON);
        // Button x = new JoystickButton(driverController, XboxConstants.X_BUTTON);
        // Button y = new JoystickButton(driverController, XboxConstants.Y_BUTTON);

        // interact with buttons
        lb.whileHeld(elevatorDownCommand);
        rb.whileHeld(elevatorUpCommand);
        a.whenHeld(autoAimCommand);
        // a.whileHeld(shootCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // drive back with 3 meter or 4 meter, aim, shoot
        return new SequentialCommandGroup(
                shuffleBoardHelper.getSelectedCommand(),
                autoAimCommand.withTimeout(2),
                shootCommand.withTimeout(5));
    }

    public ShuffleBoardHelper getShuffleBoardHelper() {
        return shuffleBoardHelper;
    }

    public XboxController getDriverController() {
        return driverController;
    }
}
