package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        //Swerve
        public static final SparkMaxConfig FrontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig FrontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig RearRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig RearLeftDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        
        static {      

            double d_turningFactor = 2 * Math.PI;
            double d_drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
                    / ModuleConstants.k_DrivingMotorReduction;
            double d_drivingVelocityFeedForward = 1/ModuleConstants.k_DriveWheelFreeSpeedRps;

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

            FrontRightDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);

            FrontRightDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor) //Meters
                .velocityConversionFactor(d_drivingFactor / 60.0);

            FrontRightDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.00,0,0) //Placeholder numbers, may need to change later.
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1,1);


            FrontLeftDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);

            FrontLeftDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor) //Meters
                .velocityConversionFactor(d_drivingFactor / 60.0);

            FrontLeftDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.00,0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1,1);

            RearRightDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);

            RearRightDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor) //Meters
                .velocityConversionFactor(d_drivingFactor / 60.0);
            
            RearRightDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.00,0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1,1);

            RearLeftDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            
            RearLeftDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor) //Meters
                .velocityConversionFactor(d_drivingFactor);
 
            RearLeftDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.00,0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1, 1);

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(d_turningFactor) //Radians
                .velocityConversionFactor(d_turningFactor / 60.0); //Radians per second.

            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1,0,0)
                .outputRange(-1,1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);
        }
    }

}