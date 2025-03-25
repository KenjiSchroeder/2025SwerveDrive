
package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
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

    private boolean AbsoluteEncoderReversed;

  
    public MAXSwerveModule(int p_driveID, int p_turnID, MotorLocation p_motorLocation, double p_chassisAngularOffset, boolean p_driveEncoderInverted, SparkMaxConfig drivingConfig) 
    {
        m_driveMotor = new SparkMax(p_driveID, MotorType.kBrushless);
        m_turnMotor = new SparkMax(p_turnID, MotorType.kBrushless);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder();
        
        m_drivingClosedLoopController = m_driveMotor.getClosedLoopController();
        m_turningClosedLoopController = m_turnMotor.getClosedLoopController();
    
        m_motorLocation = p_motorLocation;

       m_driveMotor.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters;

        m_chassisAngularOffset = p_chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
        if(p_driveEncoderInverted) {
            m_driveEncoderInverted = -1.0;
        } else {
            m_driveEncoderInverted = 1.0;
        }
        m_driveEncoder.setPosition(0);

        public swerveModuleState getState() {

        }
    }
}