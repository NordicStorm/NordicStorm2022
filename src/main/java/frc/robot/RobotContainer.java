// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoWithInit;
import frc.robot.commands.BallAutonomous;
import frc.robot.commands.FollowBall;
import frc.robot.commands.OperatorControl;
import frc.robot.commands.PathAuto;
import frc.robot.commands.ShootingUtil;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Climbers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivetrain drivetrain = new Drivetrain();
    private final Climbers climbers = new Climbers();
    private final Barrel barrel = new Barrel();

    private final Vision vision = new Vision();

    public static final Joystick leftJoystick = new Joystick(1);
    public static final Joystick rightJoystick = new Joystick(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain.setOtherSubsystems(vision, climbers, barrel);
        barrel.setOtherSubsystems(drivetrain, climbers, vision);
        climbers.setOtherSubsystems(drivetrain, barrel, vision);
        vision.setOtherSubsystems(drivetrain, climbers, barrel);
        ShootingUtil.setSubsystems(drivetrain, barrel, vision);
        // Configure the button bindings
        configureButtonBindings();
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, new OperatorControl(drivetrain, barrel, climbers, vision));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(leftJoystick, 9).whenPressed(new InstantCommand(){
            @Override
            public void execute() {
                vision.toggleLight();
            }
            @Override
            public boolean runsWhenDisabled() {
              return true;
            }
          });
        new JoystickButton(rightJoystick, 1).whileHeld(new FollowBall(drivetrain, barrel, true, false, 2.5, drivetrain.myBallColor, 2));
        new JoystickButton(rightJoystick, 7).whenPressed(new InstantCommand(){
            @Override
            public void initialize() {
                MultiPartPath path = new MultiPartPath(drivetrain);
                path.addWaypoint(3, 4);
                path.stop();
                path.finalizePath().schedule();
            }
        }, true);
        new JoystickButton(leftJoystick, 1).whenPressed(new TurnAndShoot(drivetrain, barrel, vision, 9999999999l));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        AutoWithInit auto = new BallAutonomous(drivetrain, barrel, vision);
        auto.initializeCommands();
        return auto;
    }
}
