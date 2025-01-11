package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MathUtil;
import frc.robot.Constants.FieldConstants;

public class VisionSubsystem implements Runnable {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    // private Matrix<N3, N1> curStdDevs;

    public VisionSubsystem(String cameraName, Transform3d robotToCamera) {
        photonCamera = new PhotonCamera(cameraName);
        PhotonPoseEstimator photonPoseEstimator = null;

        try {
            var tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            photonPoseEstimator = photonCamera != null
                    ? new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            robotToCamera)
                    : null;
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }

        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        try {
            if (photonCamera != null) {

                var photonResults = photonCamera.getLatestResult();

                if (photonPoseEstimator != null) {
                    if (photonResults.hasTargets()) {
                        // Updates the pose estimator
                        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                            var estimatedPose = estimatedRobotPose.estimatedPose;

                            /**
                             * If present then makes sure the measurement is on the field and
                             * sets the atomic estimated pose to the current estimated pose
                             * from PhotonPoseEstimator
                             */
                            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH
                                    && estimatedPose.getY() > 0.0
                                    && estimatedPose.getY() <= FieldConstants.WIDTH
                                    && MathUtil.isWithinTolerance(estimatedPose.getZ(), 0, 0.1)) {
                                atomicEstimatedRobotPose.set(estimatedRobotPose);
                            }

                        });
                    }
                }
            }
        } catch (

        Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public PhotonCamera getCamera() {
        return photonCamera;
    }

    public String getCameraName() {
        return photonCamera.getName();
    }

    public PhotonPoseEstimator getPhotonPoseEstimator() {
        return photonPoseEstimator;
    }

    public boolean hasTargets() {
        return photonCamera.getLatestResult().hasTargets();
    }

    public PhotonPipelineResult getLatestResults() {
        return photonCamera.getLatestResult();
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    //     Optional<EstimatedRobotPose> visionEst = Optional.empty();
    //     for (var change : pho.getAllUnreadResults()) {
    //         visionEst = photonEstimator.update(change);
    //         updateEstimationStdDevs(visionEst, change.getTargets());
    //
    //       
    //     }
    //     return visionEst;
    // }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    // private void updateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         curStdDevs = kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty())
    //                 continue;
    //             numTags++;
    //             avgDist += tagPose
    //                     .get()
    //                     .toPose2d()
    //                     .getTranslation()
    //                     .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             curStdDevs = kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1)
    //                 estStdDevs = kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else
    //                 estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             curStdDevs = estStdDevs;
    //         }
    //     }
    // }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    // public Matrix<N3, N1> getEstimationStdDevs() {
    //     return curStdDevs;
    // }

}
