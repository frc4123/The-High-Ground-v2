package frc.robot.utils;

import frc.robot.Constants.AutoAimConstants;

import org.photonvision.PhotonCamera;

public class Vision {
    // seperate class for ease of use.
    public final PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
}
