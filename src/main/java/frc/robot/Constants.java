package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //put robot constants here

    public static final double maxSpeed  = Units.feetToMeters(6.5);

    public static class OperatorConstants {
        public static final double DEADBAND        = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
    }
}
