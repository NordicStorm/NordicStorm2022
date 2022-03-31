package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public abstract class AutoWithInit extends SequentialCommandGroup{
    public abstract void initializeCommands();
}