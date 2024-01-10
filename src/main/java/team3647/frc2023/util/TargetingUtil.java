package team3647.frc2023.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class TargetingUtil {

    Pose2d speakerPose;
    Supplier<Pose2d> drivePose;

    public TargetingUtil(Pose2d speakerPose, Supplier<Pose2d> drivePose) {
        this.speakerPose = speakerPose;
        this.drivePose = drivePose;
    }

    public double angleToSpeaker() {
        var currentPose = drivePose.get();
        var toSpeaker =
                VecBuilder.fill(
                        speakerPose.getX() - currentPose.getX(),
                        speakerPose.getY() - currentPose.getY());
        var driveOrientation =
                VecBuilder.fill(
                        Math.cos(currentPose.getRotation().getRadians()),
                        Math.sin(currentPose.getRotation().getRadians()));
        double angle = Math.acos(toSpeaker.dot(driveOrientation) / toSpeaker.norm());
        double fieldAngle =
                Math.acos(toSpeaker.dot(VecBuilder.fill(-1, 0)) / toSpeaker.norm())
                        * Math.signum(currentPose.getY() - speakerPose.getY());
        boolean shouldBeNegative =
                Math.sin(currentPose.getRotation().getRadians() - fieldAngle) > 0;
        int negative = shouldBeNegative ? -1 : 1;
        SmartDashboard.putNumber("angle", Units.radiansToDegrees(angle) * negative);
        return angle * negative;
    }
}
