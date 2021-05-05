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
import frc.robot.subsystems.IntakeDrawSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShuffleBoardHelper;
import frc.robot.utils.Vision;

public class RobotContainer {
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/smartdashboard/setting-robot-preferences-from-smartdashboard.html
    private final XboxController driverController =
            new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeWheelsSubsystem intakeWheelsSubsystem = new IntakeWheelsSubsystem();
    private final IntakeDrawSubsystem intakeDrawSubsystem = new IntakeDrawSubsystem();

    private final Vision vision = new Vision();
    private final ShuffleBoardHelper shuffleBoardHelper;

    private final AutoAimCommand autoAimCommand =
            new AutoAimCommand(
                    driveSubsystem,
                    vision,
                    this,
                    () -> -driverController.getY(GenericHID.Hand.kLeft));

    private final StartEndCommand elevatorDownCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorVelo(-0.25),
                    () -> elevatorSubsystem.setElevatorVelo(0),
                    elevatorSubsystem);

    private final StartEndCommand elevatorUpCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorVelo(0.25),
                    () -> elevatorSubsystem.setElevatorVelo(0),
                    elevatorSubsystem);

    private final StartEndCommand shootCommand =
            new StartEndCommand(
                    () -> shooterSubsystem.setShooterVelo(0.25),
                    () -> shooterSubsystem.setShooterVelo(0),
                    shooterSubsystem);

    private final StartEndCommand intakeDrawUpCommand =
            new StartEndCommand(
                    () -> intakeDrawSubsystem.setIntakeDrawVelo(0.25),
                    () -> intakeDrawSubsystem.setIntakeDrawVelo(0.0),
                    intakeDrawSubsystem);

    private final StartEndCommand intakeDrawDownCommand =
            new StartEndCommand(
                    () -> intakeDrawSubsystem.setIntakeDrawVelo(0.25),
                    () -> intakeDrawSubsystem.setIntakeDrawVelo(0.0),
                    intakeDrawSubsystem);

    private final StartEndCommand intakeWheelsCommand =
            new StartEndCommand(
                    () -> intakeWheelsSubsystem.setIntakeWheelsVelo(0.80),
                    () -> intakeWheelsSubsystem.setIntakeWheelsVelo(0.0),
                    intakeWheelsSubsystem);

    private void calibrate() {
        System.out.println("Gyro is calibrating...");
        driveSubsystem.calibrateGyro();
    }

    /**
     * The {@link RobotContainer} object. Instantiates button bindings and sets the default command.
     *
     * <p>You should set up all configuration of the robot subsystems, commands, and helper
     * functions in this class as if this were a typical "Main" class. Make sure you only make one
     * object for these, and use getters as necessary.
     */
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
        Button b = new JoystickButton(driverController, XboxConstants.B_BUTTON);
        Button x = new JoystickButton(driverController, XboxConstants.X_BUTTON);
        Button y = new JoystickButton(driverController, XboxConstants.Y_BUTTON);
        // POVButton povUp = new POVButton(driverController, 0);
        // POVButton povDown = new POVButton(driverController, 180);
        // POVButton povLeft = new POVButton(driverController, 90);
        // POVButton povRight = new POVButton(driverController, 270);

        // interact with buttons
        lb.whileHeld(elevatorDownCommand);
        rb.whileHeld(elevatorUpCommand);
        a.whenHeld(autoAimCommand);
        b.whileHeld(intakeWheelsCommand);
        // TODO see how long it takes to fall down and update values accordingly
        x.whenPressed(intakeDrawDownCommand.withTimeout(2));
        y.whenPressed(intakeDrawUpCommand.withTimeout(2));
        // povUp.whenPressed(command);
        // povDown.whenPressed(command);
    }

    /**
     * Used to pass the autonomous {@link Command} to the main {@link Robot} class.
     *
     * @return the {@link Command} or CommandGroup to run in autonomous
     */
    public Command getAutonomousCommand() {
        // drive back with 3 meter or 4 meter, aim, shoot
        return new SequentialCommandGroup(
                shuffleBoardHelper.getSelectedCommand(),
                autoAimCommand.withTimeout(2),
                shootCommand.withTimeout(5));
    }

    /**
     * This {@link #shuffleBoardHelper}.
     *
     * @return this {@link #shuffleBoardHelper}
     */
    public ShuffleBoardHelper getShuffleBoardHelper() {
        return shuffleBoardHelper;
    }

    /**
     * This {@link #driverController}. This is an {@link XboxController} that is used by the main
     * driver of the robot.
     *
     * @return this {@link #driverController}
     */
    public XboxController getDriverController() {
        return driverController;
    }
}
