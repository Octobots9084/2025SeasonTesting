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
    private PIDController gyroPIDController;


    public AlignToTarget() {
        this.alignVision = AlignVision.getInstance();
        this.swerve = SwerveSubsystem.getInstance();
        this.pidController = new PIDController(4.5,0,0);
        this.lidarPIDController = new PIDController(4,0,0);
        this.gyroPIDController = new PIDController(2,0,0);
        this.gyroPIDController.enableContinuousInput(0, 2*Math.PI);
        SmartDashboard.putNumber("Set P", 2);
    }

    @Override
    public void execute() {
        gyroPIDController.setP(SmartDashboard.getNumber("Set P", 2));

        try {
            if (alignVision.getToTarget() != null) {
                speed = -pidController.calculate(alignVision.getToTarget().getY(), 0);
                SmartDashboard.putNumber("Align Command Y", alignVision.getToTarget().getY());
                SmartDashboard.putNumber("LIDAR pos", alignVision.getLidarDistance());
                SmartDashboard.putNumber("Gyro pos", swerve.getGyro());

                SmartDashboard.putNumber("PIDED Speed", speed);
            } else {
                speed = 0;
            }
        } catch (Exception e) {
            speed = 0;
        }
        lidarSpeed = -lidarPIDController.calculate(alignVision.getLidarDistance(), .2);
        SmartDashboard.putNumber("LidarDistance", alignVision.getLidarDistance());

        gyroSpeed = -gyroPIDController.calculate(swerve.getGyro(), 2*Math.PI);
        SmartDashboard.putNumber("PIDED Gyro Speed", gyroSpeed);

        swerve.driveRobotRelative(new ChassisSpeeds(lidarSpeed, speed, gyroSpeed));

    }
}
