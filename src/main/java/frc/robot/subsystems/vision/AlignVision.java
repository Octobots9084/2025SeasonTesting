package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlignVision extends SubsystemBase {

    private static AlignVision INSTANCE;
    
	public static AlignVision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AlignVision();
		}
		return INSTANCE;
	}
    
    private PhotonCamera cam;
    private Transform3d camToTarget;

    private CANrangeConfiguration configuration;
    private FovParamsConfigs paramsConfigs;
    private CANrange rightRange;
    private CANrange leftRange;
    private boolean hasTargets;
    private Matrix<N4, N4> Gct;
    private Matrix<N4, N4> Goc;
    private Matrix<N4, N1> rt;
    private Matrix<N4, N1> ro;
    private Transform3d transformOC;

    public AlignVision() {
        cam = new PhotonCamera("CamOne");
        leftRange = new CANrange(10);
        rightRange = new CANrange(12);

        paramsConfigs = new FovParamsConfigs();
        paramsConfigs.withFOVRangeX(6.75);
        paramsConfigs.withFOVRangeY(6.75);
        paramsConfigs.withFOVCenterX(6.75);
        paramsConfigs.withFOVCenterY(6.75);

        configuration = new CANrangeConfiguration();
        configuration.withFovParams(paramsConfigs);
        rightRange.getConfigurator().apply(configuration);
        leftRange.getConfigurator().apply(configuration);
    }

    @Override
	public void periodic() {
		List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        // cam.getAllUnreadResults().get(0).getBestTarget().bestCameraToTarget()

		if (!results.isEmpty()) {
			var result = results.get(results.size() - 1);

            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 16) {
                    hasTargets = true;
                    // this.camToTarget = target.getBestCameraToTarget().toMatrix();
                    Gct = target.getBestCameraToTarget().toMatrix();
                    transformOC = new Transform3d(0.1524, 0.3556, 0, new Rotation3d(0, 0, Math.toRadians(-35.0)));
                    Goc = transformOC.toMatrix();
                    rt = new Matrix<>(Nat.N4(), Nat.N1(), new double[]{0.381, 0.1524, 0, 1});
                    ro = Goc.times(Gct.times(rt));
                } else {
                    ro = null;
                }
            }
		} else {
            hasTargets = false;
        }

	}



    public double getAlignX() {
        return ro.getData()[0];
    }

    public double getAlignY() {
        return ro.getData()[1];
    }

    public double getRightLidarDistance() {
        return rightRange.getDistance().getValueAsDouble();
    }

    public double getLeftLidarDistance() {
        return leftRange.getDistance().getValueAsDouble();
    }

    public boolean getRightLidarDetect() {
        return rightRange.getIsDetected().getValue();
    }

    public boolean getLeftLidarDetect() {
        return leftRange.getIsDetected().getValue();
    }

    public boolean getHasTargets() {
        return hasTargets;
    }
}
