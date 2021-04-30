package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RumbleCommand extends WaitCommand {

    private final XboxController xboxController;
    private final double intensity;

    /**
     * Rumbles the specified {@code XboxController} for a set {@code timeOut}.
     *
     * @param xboxController the XboxController to Rumble
     * @param intensity the intensity to rumble the XboxController. Value is clamped between 0.0 and
     *     1.0
     * @param timeOut the amount in seconds the XboxController should rumble
     */
    public RumbleCommand(XboxController xboxController, double intensity, double timeOut) {
        super(timeOut);
        this.xboxController = xboxController;
        this.intensity = intensity;
    }

    @Override
    public void execute() {
        this.xboxController.setRumble(RumbleType.kLeftRumble, intensity);
        this.xboxController.setRumble(RumbleType.kLeftRumble, intensity);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        xboxController.setRumble(RumbleType.kLeftRumble, 0);
        xboxController.setRumble(RumbleType.kLeftRumble, 0);
    }
}
