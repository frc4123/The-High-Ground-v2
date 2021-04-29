package frc.robot.utils;

import frc.robot.Constants.AutoAimConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;

public class Vision {
    public static PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
    public static PhotonPipelineResult result;
}
