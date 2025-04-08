package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {


    public static final class MAXSwerveModule {
        
        //Go on manhunt for where drivingConfig actually goes

        public static final SparkMaxConfig FrontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig FrontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig RearRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig RearLeftDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        
        static {      

            double d_turningFactor = 2 * Math.PI;

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(d_turningFactor) // radians
                    .velocityConversionFactor(d_turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, d_turningFactor); 

            
        }
    }
}
