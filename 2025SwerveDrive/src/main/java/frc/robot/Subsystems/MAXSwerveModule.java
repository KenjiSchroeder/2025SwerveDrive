
package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Configs;
import frc.robot.Constants.DriveConstants.MotorLocation;

public class MAXSwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;


    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private final MotorLocation m_motorLocation;
    private final double m_driveEncoderInverted;
    
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int p_driveID, int p_turnID, MotorLocation p_motorLocation, double p_chassisAngularOffset, boolean p_driveEncoderInverted, SparkMaxConfig drivingConfig, SparkMaxConfig turningConfig) 
    {
        m_driveMotor = new SparkMax(p_driveID, MotorType.kBrushless);
        m_turnMotor = new SparkMax(p_turnID, MotorType.kBrushless);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder();
        
        m_drivingClosedLoopController = m_driveMotor.getClosedLoopController();
        m_turningClosedLoopController = m_turnMotor.getClosedLoopController();
    
        m_motorLocation = p_motorLocation;

       m_driveMotor.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_chassisAngularOffset = p_chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
        if(p_driveEncoderInverted) {
            m_driveEncoderInverted = -1.0;
        } else {
            m_driveEncoderInverted = 1.0;
        }
        m_driveEncoder.setPosition(0);

    }

    public double getDriveVelocity(){
        return m_driveEncoder.getVelocity();
    }

    public SwerveModuleState getState(){
        
        return new SwerveModuleState(
            getDriveVelocity(),
              new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
    }

    public double getDrivePosition(){
        return m_driveEncoderInverted * m_driveEncoder.getPosition();
    }

    public double getTurnPosition(){
        return m_turnEncoder.getPosition();
    }

    public double getTurnVelocity(){
        return m_turnEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));

    }

    //Actually gives a use for the setDesiredState method 
    //and allows us to acces m_desiredState outside of this subsystem
    public SwerveModuleState getDesiredState(){
        return m_desiredState;
    }

    public void setDesiredState(SwerveModuleState p_desiredState){
        //Apply angular offset of the robot to the desired st
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = p_desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = p_desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        //Optimize the reference state so that it does not spin more than 90 degrees
        correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));

        //Actually set the desired state to the new setpoint
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
        
        //In their code, they set m_desiredState to p_desiredState, but it is wrong
        m_desiredState = correctedDesiredState;
    }

    
    public void updateSmartDashboard() {
        // Position of Drive and Turn Motors
        SmartDashboard.putNumber(m_motorLocation + " driver encoder", getDrivePosition());
        SmartDashboard.putNumber(m_motorLocation + " driver velocity", getDriveVelocity());
        SmartDashboard.putNumber(m_motorLocation + " turn encoder", m_turnEncoder.getPosition());
    }
    
    public void resetEncoder(){
        m_driveEncoder.setPosition(0);
    }

    public void stopMotors(){
        m_driveMotor.stopMotor();
        m_turnMotor.stopMotor();
    }
    
}