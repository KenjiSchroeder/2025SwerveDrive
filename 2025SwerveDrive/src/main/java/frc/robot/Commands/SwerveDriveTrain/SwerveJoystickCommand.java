package frc.robot.Commands.SwerveDriveTrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Subsystems.DriveSubsystem;


public class SwerveJoystickCommand extends Command { 
    private final DriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final String statusName;

public SwerveJoystickCommand(DriveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction, String statusName) {
this.swerveSubsystem = swerveSubsystem;
this.xSpdFunction = xSpdFunction;
this.ySpdFunction = ySpdFunction;
this.turningSpdFunction = turningSpdFunction;
this.fieldOrientedFunction = fieldOrientedFunction;
this.statusName = statusName;
this.xLimiter = new SlewRateLimiter(TeleConstants.MaxAccelerationMetersPerSecondSquared);
this.yLimiter = new SlewRateLimiter(TeleConstants.MaxAccelerationMetersPerSecondSquared);
this.turningLimiter = new SlewRateLimiter(TeleConstants.MaxAngularSpeedRadiansPerSecondSquared);
addRequirements(swerveSubsystem);
}

@Override
public void initialize() {
    SmartDashboard.putString("Drive Mode", statusName);
}

@Override
public void execute(){
    double xSpeed = OIConstants.DRIVER_X_AXIS_INVERTED * xSpdFunction.get();
    double ySpeed = OIConstants.DRIVER_Y_AXIS_INVERTED * ySpdFunction.get();
    double turningSpeed = OIConstants.DRIVER_ROT_AXIS_INVERTED * turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > OIConstants.DriveDeadband ? xSpeed : 0.0; //adjust value later
    ySpeed = Math.abs(ySpeed) > OIConstants.DriveDeadband ? ySpeed : 0.0; //adjust value later
    turningSpeed = Math.abs(turningSpeed) > OIConstants.DriveDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * TeleConstants.MaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * TeleConstants.MaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * TeleConstants.MaxAngularSpeedRadiansPerSecondSquared;

    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,turningSpeed, swerveSubsystem.getRotation2d());
    } else {
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

   if(DebuggingConstants.SWERVE_DRIVE_MASTER) {
        SmartDashboard.putString("joystick", "X : " + xSpdFunction.get() + "Y : " + ySpdFunction.get() + "Theta : " + turningSpdFunction.get());
        SmartDashboard.putString("ChassisSpeeds", "X : " + chassisSpeeds.vxMetersPerSecond + "Y : " + chassisSpeeds.vyMetersPerSecond + "Theta : " + chassisSpeeds.omegaRadiansPerSecond);
   }
}

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
