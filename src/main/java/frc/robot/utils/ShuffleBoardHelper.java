package frc.robot.utils;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

import java.util.Map;

public class ShuffleBoardHelper {
    private final ThreeMeterAuto threeMeterAuto;
    private final FourMeterAuto fourMeterAuto;
    private final ShuffleboardTab driverTab;
    private final SimpleWidget isCameraCentredWidget;
    private final SimpleWidget pdpWidget;
    private final ComplexWidget cameraStreamWidget;
    private final HttpCamera cameraStream;

    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /**
     * Use to create all {@code ShuffleBoard} widgets for the robot.
     *
     * @param driveSubsystem the drive subsystem
     */
    public ShuffleBoardHelper(DriveSubsystem driveSubsystem) {
        driverTab = Shuffleboard.getTab("Driver Board");

        fourMeterAuto = new FourMeterAuto(driveSubsystem);
        threeMeterAuto = new ThreeMeterAuto(driveSubsystem);
        cameraStream =
                new HttpCamera(
                        "Photon Vision", "http://10.41.23.33:1182/stream.mjpg?1619731336501");

        driverTab.add("Select program for auto", chooser).withPosition(0, 0).withSize(2, 1);
        chooser.setDefaultOption("3 meter", threeMeterAuto.getCommand());
        chooser.addOption("4 meter", fourMeterAuto.getCommand());

        isCameraCentredWidget =
                driverTab
                        .add("Is target centered?", false)
                        .withWidget(BuiltInWidgets.kBooleanBox)
                        .withPosition(0, 1)
                        .withSize(2, 2);
        cameraStreamWidget =
                driverTab
                        .add(cameraStream)
                        .withWidget(BuiltInWidgets.kCameraStream)
                        .withPosition(2, 0)
                        .withSize(5, 4)
                        .withProperties(Map.of("Show controls", false));
        pdpWidget =
                driverTab
                        .add("PDP", BuiltInWidgets.kPowerDistributionPanel)
                        .withPosition(0, 7)
                        .withSize(3, 2);
    }

    public Command getSelectedCommand() {
        return chooser.getSelected();
    }

    public SimpleWidget getIsCameraCentredWidget() {
        return isCameraCentredWidget;
    }

    public ComplexWidget getCameraStreamWidget() {
        return cameraStreamWidget;
    }

    public SimpleWidget getPdpWidget() {
        return pdpWidget;
    }
}
