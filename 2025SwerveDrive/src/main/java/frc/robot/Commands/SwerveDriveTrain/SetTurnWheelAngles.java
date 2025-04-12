package frc.robot.Commands.SwerveDriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class SetTurnWheelAngles extends Command{
    private final DriveSubsystem swerveSubsystem;
    private final double angle;

    public SetTurnWheelAngles(DriveSubsystem swerveSubsystem, double angle){
        this.swerveSubsystem = swerveSubsystem;
        this.angle = angle;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", "TurnWheels/" + angle);
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle)); //Adjust (velocity, angle) if needed
        moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle)); //Adjust (velocity, angle) if needed
        moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle)); //Adjust (velocity, angle) if needed
        moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle)); //Adjust (velocity, angle) if needed
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
