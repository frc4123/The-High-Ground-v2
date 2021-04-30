// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class DriveSubsystem extends SubsystemBase {

    // private final WPI_TalonFX leftMaster = new WPI_TalonFX(CanIdConstants.LEFT_MASTER_ID);
    // private final WPI_TalonFX rightMaster = new WPI_TalonFX(CanIdConstants.RIGHT_MASTER_ID);
    // private final WPI_TalonFX leftSlave = new WPI_TalonFX(CanIdConstants.LEFT_SLAVE_ID);
    // private final WPI_TalonFX rightSlave = new WPI_TalonFX(CanIdConstants.RIGHT_SLAVE_ID);

    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(CanIdConstants.LEFT_MASTER_ID);
    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(CanIdConstants.RIGHT_MASTER_ID);
    private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(CanIdConstants.LEFT_SLAVE_ID);
    private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(CanIdConstants.RIGHT_SLAVE_ID);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    private final DifferentialDrive differentialDrive =
            new DifferentialDrive(leftMaster, rightMaster);

    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(gyro.getRotation2d());

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    TrajectoryConstants.SIMPLE_MOTOR_FEED_FOWARD,
                    TrajectoryConstants.DRIVE_KINEMATICS,
                    10);

    private final PIDController ramseteController = new PIDController(TrajectoryConstants.KP, 0, 0);

    /**
     * Configuration for a {@code Trajectory}. See {@see TrajectoryConstraint} for a list of
     * contraint decorators.
     */
    private final TrajectoryConfig config =
            new TrajectoryConfig(
                            TrajectoryConstants.MAX_VELOCITY, TrajectoryConstants.MAX_ACCELERATION)
                    .setKinematics(TrajectoryConstants.DRIVE_KINEMATICS)
                    .addConstraint(autoVoltageConstraint);

    /** List of {@code Trajectorys} */
    public final List<Trajectory> pathList;

    // talonfx
    // private final SupplyCurrentLimitConfiguration currentLimit =
    //     new SupplyCurrentLimitConfiguration(true, 40, 60, 1);

    public DriveSubsystem() {
        // talonfx
        // leftMaster.configSupplyCurrentLimit(currentLimit);
        // rightMaster.configSupplyCurrentLimit(currentLimit);
        // leftSlave.configSupplyCurrentLimit(currentLimit);
        // rightSlave.configSupplyCurrentLimit(currentLimit);

        // leftMaster.configSelectedFeedbackSensor(
        //     TalonFXFeedbackDevice.IntegratedSensor, 0, DriveConstants.TIMEOUT);
        // rightMaster.configSelectedFeedbackSensor(
        //     TalonFXFeedbackDevice.IntegratedSensor, 0, DriveConstants.TIMEOUT);

        // talonsrx + victorspx
        leftMaster.configContinuousCurrentLimit(10, 0);
        leftMaster.configPeakCurrentLimit(15, 0);
        leftMaster.configPeakCurrentDuration(100, 0);
        leftMaster.enableCurrentLimit(true);

        rightMaster.configContinuousCurrentLimit(40, 0);
        rightMaster.configPeakCurrentLimit(60, 0);
        rightMaster.configPeakCurrentDuration(500, 0);
        rightMaster.enableCurrentLimit(true);

        leftMaster.configSelectedFeedbackSensor(
                TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, DriveConstants.TIMEOUT);
        rightMaster.configSelectedFeedbackSensor(
                TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, DriveConstants.TIMEOUT);

        // both
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);

        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);

        pathList =
                List.of(
                        TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(1, 0)),
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config),
                        TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(2, 0)),
                                new Pose2d(4, 0, new Rotation2d(0)),
                                config));

        resetEncoders();
        // configopenloopramp() ?
    }

    /**
     * Arcade style drive. Values are squared to decrease sensitivity at low values. Please use the
     * left stick for translation and the right stick for rotation.
     *
     * <p>See {@link DifferentialDrive#curvatureDrive()} for another good drive system.
     *
     * @param fwd the translation component
     * @param rot the rotation component
     */
    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Tank drive. Typically used for trajectory following.
     *
     * @param leftVolts the voltage to supply to the left side of the drive train
     * @param rightVolts the voltage to supply to the right side of the drive train
     */
    public void voltDrive(double leftVolts, double rightVolts) {
        // TODO make sure this is volage compensated
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(-rightVolts);
        differentialDrive.feed();
    }
    /**
     * Returns the {@code DifferentialDrive} object.
     *
     * @return the {@link DifferentialDrive} instance
     */
    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    // public double getGyroAngle() {
    //     return gyro.getAngle();
    // }
    // Sometimes a gyro overrides getRotation and returns gyro as clockwise negative (which is
    // proper)
    // but their getAngle() returns clockwise positive. This means that
    // gyro.getRotation2d().getDegrees will return clockwise negative, but getAngle() would return
    // clockwise positive
    // but I'm not sure of a gyro that does this

    /**
     * Returns rotaion in degrees. Returns the rotation of the gyro in degrees, with CCW rotation
     * being positive.
     *
     * @return the heading of the gyro
     */
    public double getHeading() {
        // make sure this is negated if necessary
        return gyro.getRotation2d().getDegrees();
    }

    /** Calibrates the gyro. */
    public void calibrateGyro() {
        gyro.calibrate();
    }

    /** Sets the gyro to a heading of 0 degrees. */
    public void resetGyro() {
        gyro.reset();
    }

    /** Sets drive train encoders to 0 meters. */
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, DriveConstants.TIMEOUT);
        rightMaster.setSelectedSensorPosition(0, 0, DriveConstants.TIMEOUT);
    }

    /**
     * Resets the specified pose. Resets the drive train encoders and the {@link Pose2d} instance.
     *
     * @param pose the Pose2d instance to reset
     */
    public void resetPose(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Returns a {@code DifferentialDriveWheelSpeeds} object.
     *
     * @return a {@link DifferentialDriveWheelSpeeds} object
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getRightWheelSpeed());
    }

    /**
     * Returns the position, in meters, of the left side of the wheelbase. Position accumulates
     * stating from when the robot is turned on.
     *
     * @return the current position, in meters, of the left side of the wheelbase
     */
    private double getLeftWheelPosition() {
        // TODO remove gear ratio when using talonsrxs
        return ((leftMaster.getSelectedSensorPosition()
                        * DriveConstants.WHEEL_CIRCUMFERENCE_METERS
                        / DriveConstants.TALONFX_ENCODER_CPR)
                / DriveConstants.GEAR_RATIO);
    }

    /**
     * Returns the position, in meters, of the right side of the wheelbase. Position accumulates
     * stating from when the robot is turned on.
     *
     * @return the current position, in meters, of the right side of the wheelbase
     */
    private double getRightWheelPosition() {
        return ((leftMaster.getSelectedSensorPosition()
                        * DriveConstants.WHEEL_CIRCUMFERENCE_METERS
                        / DriveConstants.TALONFX_ENCODER_CPR)
                / DriveConstants.GEAR_RATIO);
    }

    /**
     * Returns the current velocity, in meters per second, of the left side of the wheelbase.
     *
     * @return the current velocity, in meters per second, of the left side of the wheelbase
     */
    private double getLeftWheelSpeed() {
        return (leftMaster.getSelectedSensorVelocity(0)
                * 10
                / DriveConstants.TALONFX_ENCODER_CPR
                / DriveConstants.GEAR_RATIO
                * DriveConstants.WHEEL_CIRCUMFERENCE_METERS);
    }

    /**
     * Returns the current velocity, in meters per second, of the right side of the wheelbase.
     *
     * @return the current velocity, in meters per second, of the right side of the wheelbase
     */
    private double getRightWheelSpeed() {
        return (leftMaster.getSelectedSensorVelocity(0)
                * 10
                / DriveConstants.TALONFX_ENCODER_CPR
                / DriveConstants.GEAR_RATIO
                * DriveConstants.WHEEL_CIRCUMFERENCE_METERS);
    }

    /**
     * Returns the {@code Pose2d} instance.
     *
     * @return the {@link Pose2d} instance
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    /**
     * Returns a {@code RamseteCommand} object. Used to follow the speifided {@link Trajectory}.
     *
     * @param path the {@code Trajectory} to follow
     * @return a {@link RamseteCommand} object
     */
    public RamseteCommand ramseteCommand(Trajectory path) {
        return new RamseteCommand(
                path,
                odometry::getPoseMeters,
                new RamseteController(),
                TrajectoryConstants.SIMPLE_MOTOR_FEED_FOWARD,
                TrajectoryConstants.DRIVE_KINEMATICS,
                this::getWheelSpeeds,
                ramseteController,
                ramseteController,
                this::voltDrive,
                this);
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getLeftWheelPosition(), getRightWheelPosition());
    }
}
