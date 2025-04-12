
package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


/**/

public final class Constants {

  public static final class DebuggingConstants {

      public static final boolean IS_DEBUG_MASTER = true;
      public static final boolean SWERVE_DRIVE_MASTER = true && IS_DEBUG_MASTER;
      public static final boolean IS_IN_AIR = true && IS_DEBUG_MASTER;

  }

  public static final class DriveConstants {

        public static final double k_MaxSpeedMetersPerSecond = 0.00;
        public static final double k_MaxAngularSpeed = 2 * Math.PI; //rad per sec
        //Change placeholder values into the
        //allowed maximum speed during programming
        //There needs to be more stuff here that

        //Angular offsets of modules to chassis
        public static final double FrontLeftChassisAngularOffset = 0;
        public static final double FrontRightChassisAngularOffset = 0;
        public static final double RearLeftChassisAngularOffset = 0;
        public static final double RearRightChassisAngularOffset = 0;

        //SPARK CAN MAX IDs
  
        //Change when actual robot
        public static final int FrontLeftDrivingCanId = 0;
        public static final int RearLeftDrivingCanId = 1;
        public static final int FrontRightDrivingCanId = 2;
        public static final int RearRightDrivingCanId = 3;

        public static final int FrontLeftTurningCanId = 4;
        public static final int RearLeftTurningCanId = 5;
        public static final int FrontRightTurningCanId = 6;
        public static final int RearRightTurningCanId = 7;
        
        public static final boolean FrontLeftInverted = false;
        public static final boolean RearLeftInverted = false;
        public static final boolean FrontRightInverted = false;
        public static final boolean RearRightInverted = false;

        //Placeholder values for wheel base and track width
        //Change to actual values when programming
        public static final double WheelBase = 0;
        public static final double TrackWidth = 0;

        public static final SwerveDriveKinematics k_DriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2),
            new Translation2d(WheelBase / 2, -TrackWidth / 2),
            new Translation2d(-WheelBase / 2, TrackWidth / 2),
            new Translation2d(-WheelBase / 2, -TrackWidth / 2)
        );

    // Motor Names
    public enum MotorLocation {
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_LEFT,
      REAR_RIGHT
    };

  }

    /* Swerve */

    public static final class SwerveDriveConstants {

        public static final int REAR_LEFT_MOTOR_PORT_NUMBER = 2;
        public static final int REAR_RIGHT_MOTOR_PORT_NUMBER = 4;
        public static final int FRONT_LEFT_MOTOR_PORT_NUMBER = 6;
        public static final int FRONT_RIGHT_MOTOR_PORT_NUMBER = 8;

        public static final boolean REAR_LEFT_MOTOR_REVERSED = false;
        public static final boolean REAR_RIGHT_MOTOR_REVERSED = false;
        public static final boolean FRONT_LEFT_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_MOTOR_REVERSED = false;

        public static final int STATIC_REAR_LEFT_DRIVE = DebuggingConstants.IS_IN_AIR ? 0 : 0;
        public static final int STATIC_REAR_RIGHT_DRIVE = DebuggingConstants.IS_IN_AIR ? 0 : 0;
        public static final int STATIC_FRONT_LEFT_DRIVE = DebuggingConstants.IS_IN_AIR ? 0 : 0;
        public static final int STATIC_FRONT_RIGHT_DRIVE = DebuggingConstants.IS_IN_AIR ? 0 : 0;
        
    }

    public static final class SwerveTurnConstants {

        public static final int REAR_LEFT_TURN_PORT_NUMBER = 1;
        public static final int REAR_RIGHT_TURN_PORT_NUMBER = 3;
        public static final int FRONT_LEFT_TURN_PORT_NUMBER = 5;
        public static final int FRONT_RIGHT_TURN_PORT_NUMBER = 7;

        public static final boolean REAR_LEFT_TURN_REVERSE = false;
        public static final boolean REAR_RIGHT_TURN_REVERSE = false;
        public static final boolean FRONT_LEFT_TURN_REVERSE = false;
        public static final boolean FRONT_RIGHT_TURN_REVERSE = false;

        public static final int REAR_LEFT_TURN_ABSOLUTE_ENCODER = 12;
        public static final int REAR_RIGHT_TURN_ABSOLUTE_ENCODER = 9;
        public static final int FRONT_LEFT_TURN_ABSOLUTE_ENCODER = 10;
        public static final int FRONT_RIGHT_TURN_ABSOLUTE_ENCODER = 11;

        public static final boolean REAR_LEFT_TURN_ABSOLUTE_ENCODER_REVERSE = false;
        public static final boolean REAR_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSE = false;
        public static final boolean FRONT_LEFT_TURN_ABSOLUTE_ENCODER_REVERSE = false;
        public static final boolean FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSE = false;

    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROL_PORT = 0;
        public static final double DriveDeadband = 0.20;

        public static final int DRIVER_X_AXIS = 1;
        public static final int DRIVER_Y_AXIS = 0;
        public static final int DRIVER_ROT_AXIS = 4;

        public static final int DRIVER_X_AXIS_INVERTED = 1;
        public static final int DRIVER_Y_AXIS_INVERTED = -1;
        public static final int DRIVER_ROT_AXIS_INVERTED = 1;
        
    }

    public static final class TeleConstants {
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double PXController = 1;
        public static final double PYController = 1;
        public static final double PThetaController = 1;
    }

}