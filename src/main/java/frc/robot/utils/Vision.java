package frc.robot.utils;

import frc.robot.Constants.AutoAimConstants;

import org.photonvision.PhotonCamera;

public class Vision {
    /**
     * The processed PhotonVision camera. Dashboard be accessed at http://10.41.23.0:5800 when
     * connected to the robot's wi-fi.
     */
    public final PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
}
