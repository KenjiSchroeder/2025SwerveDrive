
package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        public static final double k_MaxSpeedMetersPerSecond = 0.00;
        public static final double k_MaxAngularSpeed = 2 * Math.PI; //rad per sec
        //Change placeholder values into the
        //allowed maximum speed during programming
        //There needs to be more stuff here that

        //Angular offsets of modules to chassis
        public static final double k_FrontLeftChassisAngularOffset = 0;
        public static final double k_FrontRightChassisAngularOffset = 0;
        public static final double k_BackLeftChassisAngularOffset = 0;
        public static final double k_BackRightChassisAngularOffset = 0;

        //SPARK CAN MAX IDs
  
        //Change when actual robot
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
        // Motor Names
    public enum MotorLocation {
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_LEFT,
      REAR_RIGHT
    };
    }
}