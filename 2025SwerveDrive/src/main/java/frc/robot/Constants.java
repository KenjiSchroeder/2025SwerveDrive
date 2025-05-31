
package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**/

public final class Constants {

  public static final class DebuggingConstants {

      public static final boolean IS_DEBUG_MASTER = true;
      public static final boolean SWERVE_DRIVE_MASTER = true && IS_DEBUG_MASTER;
      public static final boolean IS_IN_AIR = true && IS_DEBUG_MASTER;

  }

  public static final class DriveConstants {
      public static final int k_pigeon2Id = 0; //Change to actual pigeon ID when programming
      public static final double k_MaxSpeedMetersPerSecond = 0.00;
      public static final double k_MaxAngularSpeed = 2 * Math.PI; //rad per sec
      //Change placeholder values into the
      //allowed maximum speed during programming
      //There needs to be more stuff here that

      //Angular offsets of modules to chassis (placeholder)
      public static final double k_FrontLeftChassisAngularOffset = 0;
      public static final double k_FrontRightChassisAngularOffset = 0;
      public static final double k_RearLeftChassisAngularOffset = 0;
      public static final double k_RearRightChassisAngularOffset = 0;

      //SPARK CAN MAX IDs
      //Change when actual robot(placeholders)
      public static final int k_FrontLeftDrivingCanId = 0;
      public static final int k_RearLeftDrivingCanId = 1;
      public static final int k_FrontRightDrivingCanId = 2;
      public static final int k_RearRightDrivingCanId = 3;

      public static final int k_FrontLeftTurningCanId = 4;
      public static final int k_RearLeftTurningCanId = 5;
      public static final int k_FrontRightTurningCanId = 6;
      public static final int k_RearRightTurningCanId = 7;

      public static final boolean k_FrontLeftInverted = false;
      public static final boolean k_RearLeftInverted = false;
      public static final boolean k_FrontRightInverted = false;
      public static final boolean k_RearRightInverted = false;

      //Placeholder values for wheel base and track width
      //Change to actual values when programming
      public static final double k_WheelBase = 0;
      public static final double k_TrackWidth = 0;

      public static final SwerveDriveKinematics k_DriveKinematics = new SwerveDriveKinematics(
              new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),
              new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),
              new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),
              new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2));


      // Motor Names
      public enum MotorLocation {
          FRONT_LEFT,
          FRONT_RIGHT,
          REAR_LEFT,
          REAR_RIGHT
      };

  }
  
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    //This 13 is currently a placeholder value
    public static final int k_DrivingMotorPinionTeeth = 13;

    public static final double k_DrivingMotorFreeSpeedRps = NeoMotorConstants.k_FreeSpeedRpmNEO_V1_1 / 60;
    public static final double k_WheelDiameterMeters = Units.inchesToMeters(3);//3 is a placeholder value
    public static final double k_WheelCircumferenceMeters = k_WheelDiameterMeters * Math.PI;

    public static final double k_DrivingMotorReduction = (45.0 * 22) / (k_DrivingMotorPinionTeeth * 15);
    public static final double k_DriveWheelFreeSpeedRps = (k_DrivingMotorFreeSpeedRps * k_WheelCircumferenceMeters)
        / k_DrivingMotorReduction;
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
        
        public static final int k_DriverControllerPort = 1;

        public static final int k_driverXAxisInverted = 1;
        public static final int k_driverYAxisInverted = 1;
        public static final int k_driverRotAxisInverted = 1;

        public static final int k_driverAxisX = 1;
        public static final int k_driverAxisY = 1;
        public static final int k_driverAxisRot = 1;
        
        public static final int k_DriveDeadband = 1;
    }

    public static final class TeleConstants {
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    }
    
    
    public static final class NeoMotorConstants {
        //Currently placeholers?
        public static final double k_FreeSpeedRpmNEO_V1_1 = 5676;
        public static final double k_FreeSpeedRpmNEO_550 = 11000;
    }

    public static final class OperatingConstants {
        public static final boolean k_usingSwerveDrive = true;
        public static final boolean k_usingGyro = true;
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