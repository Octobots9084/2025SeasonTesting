package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class Vision extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    public static Vision INSTANCE;

    public Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.5,
            0.5,
            0.5);

    public final VisionSubsystem frontEstimator = new VisionSubsystem("CamOne", VisionConstants.ROBOT_TO_STINKY);
    // public final VisionSubsystem backEstimator = new VisionSubsystem("Pinky", VisionConstants.ROBOT_TO_PINKY);

    private final Notifier allNotifier = new Notifier(() -> {
        frontEstimator.run();
        // backEstimator.run();
    });

    public Vision() {
        swerveSubsystem = SwerveSubsystem.getInstance();

        allNotifier.setName("runAll");
        allNotifier.startPeriodic(0.02);
    }

    public static Vision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Vision();
        }

        return INSTANCE;
    }

    @Override
    public void periodic() {
        if (VisionConstants.USE_VISION) {
            estimatorChecker(frontEstimator);
            // estimatorChecker(backEstimator);

        } else {
            allNotifier.close();
        }
    }

    public void estimatorChecker(VisionSubsystem estimator) {
        var cameraPose = estimator.grabLatestEstimatedPose();

        if (cameraPose != null) {
            var pose2d = cameraPose.estimatedPose.toPose2d();
            SmartDashboard.putNumber(estimator.getCameraName() + " X", pose2d.getX());
            SmartDashboard.putNumber(estimator.getCameraName() + " Y", pose2d.getY());
            swerveSubsystem.addVisionReading(pose2d, cameraPose.timestampSeconds, visionMeasurementStdDevs);
        }
    }
}
