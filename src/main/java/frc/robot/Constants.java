package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //put robot constants here

    public static final double maxSpeed  = Units.feetToMeters(11.5);
    public static double slowDriveSpeed = Units.feetToMeters(6.5);
    public static double intakeMoveMaxSpeed = 0.2; //speed intake goes up or down at
    public static double intakeMoveRange = 14.2; //old: 14.2 //encoder reading difference between intake up position and intake down position (may change based on gearbox) 
    public static double intakeAngleThreshHold = 2; //tune later
    public static double intakeSpinMaxSpeed = 0.375;

    public static double hubAprilTagHeight = 1.12395; //meters, 44.25 inches
    public static double limelightHeight = 0.6096; //meters, forgot how many inches
    public static double limelightPitch = 24.6; //degrees
    public static double limelightMaxRange = 5; //feet

    public static double launcherMinSpeed = 0.55; //
    public static double launcherMaxRange = 2.9; //meters

    public static double launcherMaxSpeed = 0.825;
    public static double launcherIdleSpeed = 0.15;
    public static double launcherIntakeIdleSpeed = -0.05;

    public static double launcherMinRPM = 2950;
    public static double velocitySlope = 600;

    public static class OperatorConstants {
        //copied from YAGSL example
        // Joystick Deadband
        public static final double DEADBAND        = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
    }
}
