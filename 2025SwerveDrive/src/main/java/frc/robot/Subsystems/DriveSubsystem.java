package frc.robot.Subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Constants.DriveConstants.MotorLocation;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.FrontLeftDrivingCanId,
        DriveConstants.FrontLeftTurningCanId,
        DriveConstants.FrontLeftChassisAngularOffset,
        DriveConstants.FrontLeftInverted,
        Configs.MAXSwerveModule.FrontLeftDrivingConfig,
        MotorLocation.FRONT_LEFT);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.FrontRightDrivingCanId,
        DriveConstants.FrontRightTurningCanId,
        DriveConstants.FrontRightChassisAngularOffset,
        DriveConstants.FrontRightInverted,
        Configs.MAXSwerveModule.FrontRightDrivingConfig,
        MotorLocation.FRONT_RIGHT);
        
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        DriveConstants.RearLeftDrivingCanId,
        DriveConstants.RearLeftTurningCanId,
        DriveConstants.RearLeftChassisAngularOffset,
        DriveConstants.RearLeftInverted,
        Configs.MAXSwerveModule.RearLeftDrivingConfig,
        MotorLocation.REAR_LEFT);
        
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
        DriveConstants.RearRightDrivingCanId,
        DriveConstants.RearRightTurningCanId,
        DriveConstants.RearRightChassisAngularOffset,
        DriveConstants.RearRightInverted,
        Configs.MAXSwerveModule.RearRightDrivingConfig,
        MotorLocation.REAR_RIGHT);

    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }
    

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String statusName){
        double multiplier = 1; //Adjust value for later
        double xSpeedDelivered = xSpeed * DriveConstants.k_MaxSpeedMetersPerSecond * multiplier;
        double ySpeedDelivered = ySpeed * DriveConstants.k_MaxSpeedMetersPerSecond * multiplier;
        double rotDelivered = rot * DriveConstants.k_MaxAngularSpeed * multiplier; 

        SwerveModuleState[] swerveModuleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
            SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.k_MaxSpeedMetersPerSecond);
            SwerveModuleState[] m_desiredModuleStates = swerveModuleStates;
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            m_backRight.setDesiredState(swerveModuleStates[3]);

            SmartDashboard.putString("Drive Mode", statusName);
    }

    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.k_MaxSpeedMetersPerSecond);
            m_frontLeft.setDesiredState(desiredStates[0]);
            m_frontRight.setDesiredState(desiredStates[1]);
            m_backLeft.setDesiredState(desiredStates[2]);
            m_backRight.setDesiredState(desiredStates[3]);
            SwerveModuleState[] m_desiredModuleStates = desiredStates;
    }

    public Rotation2d getRotation2d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation2d'");
    }

    public void stopModules() {
        
    }
}
