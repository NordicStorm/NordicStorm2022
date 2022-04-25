package frc.robot.commands;

import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Drivetrain;

public class PathAuto extends AutoWithInit{

    public PathAuto(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }
    Drivetrain drivetrain;
    @Override
    public void initializeCommands() {
// !PATHWEAVER_INFO: {"trackWidth":0.762,"gameName":"Rapid React","outputDir":"C:\\Users\\Nordic Storm 3018\\FRC\\NordicStorm2022\\src\\main\\java\\frc\\robot\\commands\\PathAuto.java"}
        MultiPartPath path;
        path = new MultiPartPath(drivetrain);
        path.addWaypoint(1.444, 1.102);
        path.addWaypoint(1.311, 3.346);
        path.addWaypoint(2.674, 1.085);
        path.addWaypoint(2.823, 3.396);
        path.addWaypoint(4.801, 3.445);
        path.addWaypoint(3.737, 2.548);
        path.addWaypoint(5.084, 1.551);
        path.addWaypoint(3.904, 1.019);
        path.addStop();
        addCommands(path.finalizePath());

        
    }
    
}