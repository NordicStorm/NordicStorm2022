// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OperatorControl extends CommandBase {

  private final Drivetrain drivetrain;
  private final Vision vision;
  private final Barrel barrel;
  private final Climbers climbers;

  public OperatorControl(Drivetrain drivetrain, Barrel barrel, Climbers climbers, Vision vision) {
    this.drivetrain=drivetrain;
    this.barrel=barrel;
    this.vision=vision;
    this.climbers = climbers;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
}
