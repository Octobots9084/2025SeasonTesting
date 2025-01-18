package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.geometry.Transform3d;
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

		if (!results.isEmpty()) {
			var result = results.get(results.size() - 1);

            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    this.camToTarget = target.getBestCameraToTarget();
                } else {
                    camToTarget = new Transform3d();
                }
            }
		}

	}


    public Transform3d getToTarget() {
        return camToTarget;
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

    public int getCurrentTag() {
        return cam.getAllUnreadResults().get(cam.getAllUnreadResults().size() - 1).getTargets().get(0).getFiducialId();
    }
}
