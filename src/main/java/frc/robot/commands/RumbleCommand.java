package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RumbleCommand extends CommandBase {

    private Timer timer;
    private double timeOut;
    private XboxController xboxController;

    /**
     * Rumbles the specified XboxController for a specific time.
     *
     * @param xboxController The XboxController to Rumble
     * @param intensity The intensity to rumble the XboxController. Value is clamped between 0.0 and
     *     1.0
     * @param timeOut The amount in seconds the XboxController should rumble
     */
    public RumbleCommand(XboxController xboxController, double intensity, double timeOut) {
        this.timeOut = timeOut;
        this.xboxController = xboxController;

        timer = new Timer();
        timer.start();

        this.xboxController.setRumble(RumbleType.kLeftRumble, intensity);
        this.xboxController.setRumble(RumbleType.kLeftRumble, intensity);
    }

    @Override
    public void end(boolean interrupted) {
        xboxController.setRumble(RumbleType.kLeftRumble, 0);
        xboxController.setRumble(RumbleType.kLeftRumble, 0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= timeOut) {
            return true;
        }
        return false;
    }
}
