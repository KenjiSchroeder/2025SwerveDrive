package frc.robot.commands.SwerveDriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

//Before working on this make a DriveSubsystem
public class MoveDirection extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final String statusName;
    private final Translation2d translation;
    private Pose2d finalPosition;
    private double xVal;
    private double yVal;
    private double multiplier, threshold;

    public MoveDirection(DriveSubsystem swerveSubsystem, String statusName, Translation2d translation, double xVal, double yVal) {
        this.swerveSubsystem = swerveSubsystem;
        this.statusName = statusName;
        this.translation = translation;
        multiplier = 0.75; // Adjust
        threshold = 0.1; // Adjust
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("DriveMode", "MoveDirection/" + statusName);
        finalPosition = swerveSubsystem.getPose().plus(new Transform2d(translation, swerveSubsystem.getRotation2d()));
        double distance = Math.sqrt(translation.getX() * translation.getX() + translation.getY() * translation.getY());
        xVal = translation.getX() / distance;
        yVal = translation.getY() / distance;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Joystick", "X: " + xVal + "Y: " + yVal);
        swerveSubsystem.drive(xVal, yVal, 0, true, false);
    }
}