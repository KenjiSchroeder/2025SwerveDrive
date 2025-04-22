package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Constants.DriveConstants.MotorLocation;

import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class DriveSubsystem extends SubsystemBase {

    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.k_FrontLeftDrivingCanId,
        DriveConstants.k_FrontLeftTurningCanId,
        MotorLocation.FRONT_LEFT,
        DriveConstants.k_FrontLeftChassisAngularOffset,
        DriveConstants.k_FrontLeftInverted,
        Configs.MAXSwerveModule.FrontLeftDrivingConfig);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.k_FrontRightDrivingCanId,
        DriveConstants.k_FrontRightTurningCanId,
        MotorLocation.FRONT_RIGHT,
        DriveConstants.k_FrontRightChassisAngularOffset,
        DriveConstants.k_FrontRightInverted,
        Configs.MAXSwerveModule.FrontRightDrivingConfig);
        
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        DriveConstants.k_RearLeftDrivingCanId,
        DriveConstants.k_RearLeftTurningCanId,
        MotorLocation.REAR_LEFT,
        DriveConstants.k_RearLeftChassisAngularOffset,
        DriveConstants.k_RearLeftInverted,
        Configs.MAXSwerveModule.RearLeftDrivingConfig);
        
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
        DriveConstants.k_RearRightDrivingCanId,
        DriveConstants.k_RearRightTurningCanId,
        MotorLocation.REAR_RIGHT,
        DriveConstants.k_RearRightChassisAngularOffset,
        DriveConstants.k_RearRightInverted,
        Configs.MAXSwerveModule.RearRightDrivingConfig);
        
        private SwerveModuleState[] m_desiredModuleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

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
            m_desiredModuleStates = swerveModuleStates;
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
            m_desiredModuleStates = desiredStates;
    }

    public Rotation2d getRotation2d() {
        if(OperatingConstants.k_usingGyro){
            return new Rotation2d(m_gyro.getYaw().getValue());
        } else {
    
            return new Rotation2d(0); //value to be changed later probobly
        }
        
    }

    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_backLeft.stopMotors();
        m_backRight.stopMotors();

       m_desiredModuleStates[0].speedMetersPerSecond = 0; //To be changed later probobly
       m_desiredModuleStates[1].speedMetersPerSecond = 0; //To be changed later probobly
       m_desiredModuleStates[2].speedMetersPerSecond = 0; //To be changed later probobly
       m_desiredModuleStates[3].speedMetersPerSecond = 0; //To be changed later probobly
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.k_DriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
        );
    }
    // Limits speed from negative max speed to maxspeed inclusive
    public void testDriveMotors(double speed) {
        speed = Math.signum(speed) * Math.min(Math.abs(speed), TeleConstants.MaxAngularSpeedRadiansPerSecond);

        //Sets the speed
        m_frontLeft.testDriveMotors(speed);
        m_frontRight.testDriveMotors(speed);
        m_backLeft.testDriveMotors(speed);
        m_backRight.testDriveMotors(speed);

        //Updates the speed
        m_desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(speed, new Rotation2d(m_frontLeft.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_frontRight.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_backLeft.getTurnPosition())),
            new SwerveModuleState(speed, new Rotation2d(m_backRight.getTurnPosition()))
        };
    }
    //A function that turns only the wheels, @param pos Position to turn wheels
    public void testTurnMotors(double pos) {
        m_frontLeft.testTurnMotors(pos);
        m_frontRight.testTurnMotors(pos);
        m_backLeft.testTurnMotors(pos);
        m_backRight.testTurnMotors(pos);

        m_desiredModuleStates = new SwerveModuleState[]{
        new SwerveModuleState(m_frontLeft.getDriveVelocity(), new Rotation2d(m_frontLeft.getTurnPosition() + pos)),
        new SwerveModuleState(m_frontRight.getDriveVelocity(), new Rotation2d(m_frontRight.getTurnPosition() + pos)),
        new SwerveModuleState(m_backLeft.getDriveVelocity(), new Rotation2d(m_backLeft.getTurnPosition() + pos)),
        new SwerveModuleState(m_backRight.getDriveVelocity(), new Rotation2d(m_backRight.getTurnPosition() + pos))
        };
    }

    public void testTurnMotors(DoubleSupplier[] wheelPos, boolean turnClockwise) {
        double val = 10 * (turnClockwise ? 1 : -1);
    
        m_frontLeft.testTurnMotors(wheelPos[0].getAsDouble() + val);
        m_frontRight.testTurnMotors(wheelPos[1].getAsDouble() + val);
        m_backLeft.testTurnMotors(wheelPos[2].getAsDouble() + val);
        m_backRight.testTurnMotors(wheelPos[3].getAsDouble() + val);
    
        m_desiredModuleStates = new SwerveModuleState[]{ //Adjust speedMetersPerSecond later if needed
            new SwerveModuleState(5, new Rotation2d(wheelPos[0].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[1].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[2].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[3].getAsDouble() + val))
        };
    }

    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[]{
            () -> m_frontLeft.getTurnPosition(),
            () -> m_frontRight.getTurnPosition(),
            () -> m_backLeft.getTurnPosition(),
            () -> m_backRight.getTurnPosition()
        };

    }
}