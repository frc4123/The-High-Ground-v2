package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble {
    // Thanks to Banks for this code
    private final GenericHID controller;

    public Rumble(GenericHID controller) {
        this.controller = controller;
    }

    long lastRumbleStartMillis = 0;
    int rumbleDurationMs = 500;
    boolean rumbling = false;

    /** Checks if the controller should rumble. */
    public void periodic() {
        if (System.currentTimeMillis() - lastRumbleStartMillis >= rumbleDurationMs && rumbling) {
            setRumble(0);
            rumbling = false;
        }
    }

    private void setRumble(double power) {
        controller.setRumble(RumbleType.kLeftRumble, power);
        controller.setRumble(RumbleType.kRightRumble, power);
    }

    /**
     * Starts the rumble with the specified power and duration.
     *
     * @param power the power to rumble the controller at. Values are from -1 to 1
     * @param durationMs the duration to rumble the controller for, in milliseconds
     */
    public void startRumble(double power, int durationMs) {
        rumbleDurationMs = durationMs;
        rumbling = true;
        setRumble(power);
        lastRumbleStartMillis = System.currentTimeMillis();
    }
    /**
     * Starts the rumble with the specified power and a default time-out of 500 milliseconds.
     *
     * @param power the power to rumble the controller at. Values are from -1 to 1
     */
    public void startRumble(double power) {
        startRumble(power, rumbleDurationMs);
    }
}
