package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.math.geometry.Transform3d;
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
    private CANrange range;

    public AlignVision() {
        cam = new PhotonCamera("CamOne");
        range = new CANrange(10);

        paramsConfigs = new FovParamsConfigs();
        paramsConfigs.withFOVRangeX(6.75);
        paramsConfigs.withFOVRangeY(6.75);
        paramsConfigs.withFOVCenterX(6.75);
        paramsConfigs.withFOVCenterY(6.75);

        configuration = new CANrangeConfiguration();
        configuration.withFovParams(paramsConfigs);
        range.getConfigurator().apply(configuration);
    }

    @Override
	public void periodic() {
		List<PhotonPipelineResult> results = cam.getAllUnreadResults();

		if (!results.isEmpty()) {
			var result = results.get(results.size() - 1);

            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 6) {
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

    public double getLidarDistance() {
        return range.getDistance().getValueAsDouble();
    }
}
