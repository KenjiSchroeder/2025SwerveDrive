package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
;
public class TestMotorsSubsystem extends SubsystemBase{
    
    private final SparkMax motor;
    private final String m_name;
    private final DigitalInput m_limitSwitch;
    private final boolean m_usingLimitSwitch;
    private final boolean m_stopGoPositive;

    public TestMotorsSubsystem(String name, int id, SparkBaseConfig config) {
        m_name = name;
        motor = new SparkMax(id, MotorType.kBrushless);
        m_limitSwitch = null;
        m_usingLimitSwitch = false;
        m_stopGoPositive = false;
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public TestMotorsSubsystem(String name, int id, SparkBaseConfig config, int limitSwitchId, boolean stopGoPositive) {
        m_name = name;
        motor = new SparkMax(id, MotorType.kBrushless);
        m_limitSwitch = new DigitalInput(limitSwitchId);
        m_usingLimitSwitch = true;
        m_stopGoPositive = stopGoPositive;
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        speed = Math.signum(speed) * Math.min(Math.max(Math.abs(speed), 0), 1);
        if (m_usingLimitSwitch) {
            if(getLimitSwitch()) {
                if(m_stopGoPositive) {
                    speed = Math.max(speed, 0);
                } else {
                    speed = Math.min(speed, 0);
                }
            }
        }
        motor.set(speed);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public boolean getLimitSwitch() {
        if (m_usingLimitSwitch) {
            return !m_limitSwitch.get();
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if(m_usingLimitSwitch) {
            SmartDashboard.putBoolean(m_name + " Limit Switch", getLimitSwitch());
        }
        //Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
        // Add simulation code here if needed
    }
}
