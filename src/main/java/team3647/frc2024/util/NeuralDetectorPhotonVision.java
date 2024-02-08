package team3647.frc2024.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;

public class NeuralDetectorPhotonVision extends PhotonCamera implements NeuralDetector {
    public NeuralDetectorPhotonVision(String camera) {
        super(NetworkTableInstance.getDefault(), camera);
    }

    public double getTX() {
        var result = this.getLatestResult();
        return result.getBestTarget().getYaw();
    }

    public boolean hasTarget() {
        var result = this.getLatestResult();
        return result.hasTargets();
    }
}
