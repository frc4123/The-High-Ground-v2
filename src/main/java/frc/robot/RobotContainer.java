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

    // TODO add rumble on vision alignment
    private final XboxController driverController =
            new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // private final ThreeMeterAuto threeMeterAuto = new ThreeMeterAuto(driveSubsystem);
    // private final FourMeterAuto fourMeterAuto = new FourMeterAuto(driveSubsystem);

    // private final SendableChooser<Command> chooser = new SendableChooser<>();
    public static boolean isCameraCentred = false;
    
    ShuffleBoardHelper shuffleBoardHelper;

    private final AutoAimCommand autoAimCommand =
            new AutoAimCommand(driveSubsystem, () -> -driverController.getY(GenericHID.Hand.kLeft));

    private final StartEndCommand elevatorDownCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorSpeed(-.25),
                    () -> elevatorSubsystem.setElevatorSpeed(0),
                    elevatorSubsystem);

    private final StartEndCommand elevatorUpCommand =
            new StartEndCommand(
                    () -> elevatorSubsystem.setElevatorSpeed(.25),
                    () -> elevatorSubsystem.setElevatorSpeed(0),
                    elevatorSubsystem);

    private final StartEndCommand shootCommand =
            new StartEndCommand(
                    () -> shooterSubsystem.setSpeed(.25),
                    () -> shooterSubsystem.setSpeed(0),
                    shooterSubsystem);

    private void calibrate() {
        System.out.println("Gyro is calibrating...");
        driveSubsystem.calibrateGyro();
    }

    // private void shuffleboardSetup() {
    //     final ShuffleboardTab tab = Shuffleboard.getTab("Driver Board");

    //     tab.add("Select program for auto", chooser);
    //     chooser.setDefaultOption("3 meter", threeMeterAuto.getCommand());
    //     chooser.addOption("4 meter", fourMeterAuto.getCommand());
    // }

    public RobotContainer() {
        shuffleBoardHelper = new ShuffleBoardHelper(driveSubsystem);
        calibrate();
        // shuffleboardSetup();
        configureButtonBindings();

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
                // lambda?
                shuffleBoardHelper.getSelecteCommand(),
                autoAimCommand.withTimeout(2),
                shootCommand.withTimeout(5));
    }

    public ShuffleBoardHelper getShuffleBoardHelper(){
            return shuffleBoardHelper;
    }
}
