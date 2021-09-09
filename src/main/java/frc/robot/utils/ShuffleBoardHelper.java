package frc.robot.utils;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
    private SimpleWidget isCameraCentredWidget;
    private PowerDistributionPanel pdp;
    private ComplexWidget pdpWidget;
    private ComplexWidget cameraStreamWidget;
    private SimpleWidget distanceToTargetWidget;
    private SendableChooser<Command> chooser;
    private HttpCamera cameraStream;

    /**
     * Used to create all {@link Shuffleboard} widgets for the robot.
     *
     * @param driveSubsystem the drive subsystem
     */
    public ShuffleBoardHelper(DriveSubsystem driveSubsystem) {
        driverTab = Shuffleboard.getTab("Driver Board");

        fourMeterAuto = new FourMeterAuto(driveSubsystem);
        threeMeterAuto = new ThreeMeterAuto(driveSubsystem);
        pdp = new PowerDistributionPanel();
        cameraStream = new HttpCamera("Vision", "http://10.41.23.33:1182/?action=stream");
        chooser = new SendableChooser<>();

        setupLayout();
    }

    private void setupLayout() {
        driverTab.add("Select program for auto", chooser).withPosition(0, 0).withSize(2, 1);
        chooser.setDefaultOption("3 meter", threeMeterAuto.getCommand());
        chooser.addOption("4 meter", fourMeterAuto.getCommand());

        CameraServer.getInstance().addCamera(cameraStream);

        isCameraCentredWidget = driverTab.add("Is target centered?", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 1).withSize(2, 2);
        // this (should) get the processed footage from photon vision
        cameraStreamWidget = driverTab.add(cameraStream).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0)
                .withSize(5, 4).withProperties(Map.of("Show controls", false));
        // this gets the unprocessed camera footage.
        // cameraStreamWidget =
        // driverTab
        // .add(cameraStream)
        // .withPosition(2, 0)
        // .withSize(5, 4)
        // .withProperties(Map.of("Show controls", false));
        // still doesnt work
        pdpWidget = driverTab.add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistributionPanel).withPosition(0, 7)
                .withSize(3, 2);
        distanceToTargetWidget = driverTab.add("Distance to target", 0).withPosition(7, 2).withSize(2, 1);
    }

    /**
     * Returns the selected {@code Command} in the {@link SendableChooser} box on
     * Shuffleboard.
     *
     * @return the selected autonomous command
     */
    public Command getSelectedCommand() {
        return chooser.getSelected();
    }

    /**
     * Returns this {@link #isCameraCentredWidget}. This is a boolean box that shows
     * whether the front camera is centered on a target.
     *
     * @return this {@link #isCameraCentredWidget}
     */
    public SimpleWidget getIsCameraCentredWidget() {
        return isCameraCentredWidget;
    }

    /**
     * Returns this {@link #cameraStreamWidget}. This will either be a processed
     * {@link HttpCamera} stream or an unprocessed {@link UsbCamera} stream.
     *
     * @return this {@link cameraStreamWidget}
     */
    public ComplexWidget getCameraStreamWidget() {
        return cameraStreamWidget;
    }

    /**
     * Returns this {@link #pdpWidget}. This displays the power draw per PDP channel
     * and total voltage and amperage.
     *
     * @return this {@link #pdpWidget}
     */
    public ComplexWidget getPdpWidget() {
        return pdpWidget;
    }

    public SimpleWidget getDistanceToTargetWidget() {
        return distanceToTargetWidget;
    }
}
