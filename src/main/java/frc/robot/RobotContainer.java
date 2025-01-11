// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootIfAble;
import frc.robot.commands.vision.AlignToTarget;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.AlignVision;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private SwerveSubsystem swerveSubsystem;
    // private Vision vision;
    private AlignVision alignVision;
    
    private final SendableChooser<Command> autoChooser;


    private CommandJoystick leftJoystick;
    private CommandJoystick rightJoystick;
    private CommandJoystick buttonsJoystick;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = SwerveSubsystem.getInstance();
        // vision = Vision.getInstance();
        alignVision = AlignVision.getInstance();

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public void configureBindings() {

        leftJoystick = new CommandJoystick(0);
        rightJoystick = new CommandJoystick(1);
        buttonsJoystick = new CommandJoystick(2);

        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(() -> {
            return MathUtil.applyDeadband(-leftJoystick.getY(), 0.08);
        }, () -> {
            return MathUtil.applyDeadband(-leftJoystick.getX(), 0.08);
        }, () -> {
            return MathUtil.applyDeadband(rightJoystick.getX(), 0.08);
        }));

        buttonsJoystick.button(1).whileTrue(new AlignToTarget());
        buttonsJoystick.button(2).onTrue(new InstantCommand(()->{swerveSubsystem.zeroGyro();}));
    }
}
