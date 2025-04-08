package frc.robot.commands.SwerveDriveTrain;

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
this.xLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
this.yLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
this.turningLimiter = new SlewRateLimiter(TeleConstants.k_MaxAngularSpeedRadiansPerSecondSquared);
addRequirements(swerveSubsystem);
}

@Override
public void initialize() {
    SmartDashboard.putString("Drive Mode", statusName);
}

@Override
public void execute(){
    double xSpeed = OIConstants.k_driverXAxisInverted * xSpdFunction.get();
    double ySpeed = OIConstants.k_driverYAxisInverted * ySpdFunction.get();
    double turningSpeed = OIConstants.k_driverRotAxisInverted * turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > OIConstants.k_DriveDeadband ? xSpeed : 0.0; //adjust value later
    ySpeed = Math.abs(ySpeed) > OIConstants.k_DriveDeadband ? ySpeed : 0.0; //adjust value later
    turningSpeed = Math.abs(turningSpeed) > OIConstants.k_DriveDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * TeleConstants.k_MaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * TeleConstants.k_MaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * TeleConstants.k_MaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,turningSpeed, swerveSubsystem.getRotation2d());
    } else {
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

   if(DebuggingConstants.k_swerveDriveDebug) {
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
