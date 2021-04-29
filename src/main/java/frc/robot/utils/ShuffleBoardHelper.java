package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class ShuffleBoardHelper {
    private final ThreeMeterAuto threeMeterAuto;
    private final FourMeterAuto fourMeterAuto;
    private final ShuffleboardTab tab;
    SimpleWidget isCameraCentred;
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    public ShuffleBoardHelper(DriveSubsystem driveSubsystem) {
        fourMeterAuto = new FourMeterAuto(driveSubsystem);
        threeMeterAuto = new ThreeMeterAuto(driveSubsystem);
        tab = Shuffleboard.getTab("Driver Board");

        tab.add("Select program for auto", chooser);
        chooser.setDefaultOption("3 meter", threeMeterAuto.getCommand());
        chooser.addOption("4 meter", fourMeterAuto.getCommand());

        isCameraCentred = tab.add("Is target centered?", false).withWidget(BuiltInWidgets.kBooleanBox);
    }

    public Command getSelecteCommand() {
        return chooser.getSelected();
    }

    public SimpleWidget getIsCameraCentred(){
        return isCameraCentred;
    }
}
