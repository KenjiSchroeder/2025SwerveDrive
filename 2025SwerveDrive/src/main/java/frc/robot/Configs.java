package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    //Swerve
    public static final class MAXSwerveModule{
        
        //Go on manhunt for where drivingConfig actually goes

        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backLeftDrivingConfig = new SparkMaxConfig();

        static{

        }
    }
}
