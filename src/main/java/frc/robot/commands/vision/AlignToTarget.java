package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.AlignVision;
import frc.robot.util.MathUtil;

public class AlignToTarget extends Command {
    private final AlignVision alignVision;
    private final SwerveSubsystem swerve;
    private double speed;
    private double lidarSpeed;
    private double gyroSpeed;
    private PIDController pidController;
    private PIDController lidarPIDController;
    private PIDController cameraDepthPIDController;
    private PIDController gyroPIDController;
    private double aveLidarDist;
    private double diffLidarDist;
    private boolean usingLidar;


    public AlignToTarget() {
        this.alignVision = AlignVision.getInstance();
        this.swerve = SwerveSubsystem.getInstance();
        this.pidController = new PIDController(2,0,0);
        this.lidarPIDController = new PIDController(2,0,0);
        this.cameraDepthPIDController = new PIDController(1.25,0,0);
        this.gyroPIDController = new PIDController(4,0,0);
        this.gyroPIDController.enableContinuousInput(0, 2*Math.PI);
        SmartDashboard.putNumber("Set P", 1);
    }

    @Override
    public void execute() {
        // gyroPIDController.setP(SmartDashboard.getNumber("Set P", 1));
        aveLidarDist = (alignVision.getRightLidarDistance() + alignVision.getLeftLidarDistance()) / 2;
        usingLidar = alignVision.getRightLidarDetect() && alignVision.getLeftLidarDetect();
        diffLidarDist = alignVision.getRightLidarDistance() - alignVision.getLeftLidarDistance();

        try {
            // SmartDashboard.putNumber("Current Tag", alignVision.getCurrentTag());

            if (alignVision.getToTarget() != null && alignVision.getHasTargets()) {
                speed = pidController.calculate(alignVision.getToTarget().getY(), 0);
                lidarSpeed = usingLidar ? lidarPIDController.calculate(aveLidarDist, .12) : cameraDepthPIDController.calculate(alignVision.getToTarget().getX(), .4);
                gyroSpeed = usingLidar ? gyroPIDController.calculate(Math.asin(diffLidarDist / .605), 0) : -gyroPIDController.calculate(swerve.getGyro(), Math.toRadians(-60));

                // SmartDashboard.putNumber("Horizontal Align", alignVision.getToTarget().getY());
                // SmartDashboard.putNumber("Lidar Difference", (alignVision.getRightLidarDistance() - alignVision.getLeftLidarDistance()));
            } else {
                speed = 0;
                lidarSpeed = 0;
                gyroSpeed = 0;
            }
        } catch (Exception e) {
            speed = 0;
            lidarSpeed = 0;
            gyroSpeed = 0;
        }

        SmartDashboard.putNumber("Average Lidar Depth", aveLidarDist);

        SmartDashboard.putBoolean("BothLidarGood", usingLidar);       
        SmartDashboard.putNumber("LidarAngle", Math.asin(diffLidarDist / .605));
        SmartDashboard.putNumber("Gyro pos", swerve.getGyro());

        swerve.driveRobotRelative(new ChassisSpeeds(lidarSpeed, speed, gyroSpeed));

    }
}
