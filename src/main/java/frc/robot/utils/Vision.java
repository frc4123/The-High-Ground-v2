package frc.robot.utils;

import frc.robot.Constants.AutoAimConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;

public class Vision {
    // seperate class for ease of use. You could probably also just make these have getters and put
    // in AutoAimCommand
    public static PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
    public static PhotonPipelineResult result;
}
