package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.Timer;
import java.util.TimerTask;

public class Rumble {
    GenericHID controller;

    Timer timer;
    TimerTask task =
            new TimerTask() {

                @Override
                public void run() {
                    controller.setRumble(RumbleType.kRightRumble, 0.0);
                    controller.setRumble(RumbleType.kLeftRumble, 0.0);
                }
            };

    public Rumble() {}

    /**
     * Rumbles the specified controller for the specified amount of time.
     *
     * @param controller the controller to rumble
     * @param timeOut the time, in seconds, to rumble the controller
     * @throws IllegalArgumentException if time.getTime() is negative
     * @throws IllegalStateException if task was already scheduled or cancelled, timer was
     *     cancelled, or timer thread terminated.t
     * @throws InterruptedException if any thread interrupted the current thread before or while the
     *     current thread was waiting for a notification. The interrupted status of the current
     *     thread is cleared when this exception is thrown.
     */
    public void start(GenericHID controller, long timeOut)
            throws IllegalArgumentException, IllegalStateException, InterruptedException {
        timer = new Timer();
        this.controller = controller;

        controller.setRumble(RumbleType.kLeftRumble, 0.5);
        controller.setRumble(RumbleType.kRightRumble, 0.5);

        timer.schedule(task, timeOut * 1000);

        timer.wait();

        timer.cancel();
    }
}
