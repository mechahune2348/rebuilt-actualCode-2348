package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.util.sendable.*;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

//import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.DistanceUnit;

/**Contains robot commands that use the Limelight and/or scoring mechanisms. 
 */
public class CommandSubsystem extends SubsystemBase {
    //When using the Limelight, copy these into the methods you want to use them in (if these are put here instead of in a method the value isn't updated)
    //double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    //double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    //double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    //boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    //All of the non-drivetrain motors 
    static TalonFX intakeSpin = new TalonFX(53); //Motor that brings balls into the robot
    static SparkMax intakeDownL = new SparkMax(15,MotorType.kBrushless); //Left motor to lift intake
    static SparkMax intakeDownR = new SparkMax(14,MotorType.kBrushless); //Right motor to lift intake
    static SparkMax shooterFlywheel = new SparkMax(13,MotorType.kBrushless); //Motor that launches balls
    static SparkMax shooterIn = new SparkMax(12,MotorType.kBrushless); //Motor that brings balls into the launcher
    
    //LED Setup 
    AddressableLED led = new AddressableLED(1);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
    AddressableLEDBufferView leftSide = m_ledBuffer.createView(13, 44);
    AddressableLEDBufferView centerSide = m_ledBuffer.createView( 45, 73); 
    AddressableLEDBufferView rightSide = m_ledBuffer.createView( 74, 108).reversed();
    AddressableLEDBufferView underShooter = m_ledBuffer.createView( 109, 140);

    //Colors for LEDs
    LEDPattern redSolidPattern = LEDPattern.solid(Color.kRed);
    LEDPattern redDiscontinuous = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkRed, Color.kDarkRed, Color.kFirstRed).scrollAtAbsoluteSpeed(InchesPerSecond.of(12), Meters.of(1.5/140));
    LEDPattern blueSolidPattern = LEDPattern.solid(Color.kBlue);
    LEDPattern blueDiscontinuous = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkBlue, Color.kDarkBlue, Color.kFirstBlue).scrollAtAbsoluteSpeed(InchesPerSecond.of(12), Meters.of(1.5/140));
    LEDPattern whiteSolidPattern = LEDPattern.solid(Color.kWhite);
    LEDPattern rainbowScroll = LEDPattern.rainbow(235,255).scrollAtAbsoluteSpeed(InchesPerSecond.of(12), Meters.of(1.5/140));

    //If shooting manual, pulse in team color and have sides with a discontinuous gradient that gets faster as flywheel charges
    //If automatic, solid and get brighter as distance increases with top flashing when flywheel
    boolean isRedTeam;
    
    private final double intakeInitialPosition;

    int clockTimer = 0;
    double autoTX = 0;
    

    public CommandSubsystem(){
        clockTimer = 0;
        // var teamColor = DriverStation.getAlliance();
        // boolean isRedAlliance = teamColor.equals()
        //add more as necessary
        intakeInitialPosition = intakeDownL.getEncoder().getVelocity();

        var team = DriverStation.getAlliance(); //copied from last year's LED code
        isRedTeam = true;

        //For team colors on LEDs
        if(team.isPresent()){
            if(team.get() == DriverStation.Alliance.Red){
                redDiscontinuous.applyTo(leftSide);
                redDiscontinuous.applyTo(rightSide);
                redSolidPattern.applyTo(centerSide);
                redSolidPattern.applyTo(underShooter);
                System.out.println("RED");
                isRedTeam = true;
            } else if(team.get() == DriverStation.Alliance.Blue){
                blueDiscontinuous.applyTo(leftSide);
                blueDiscontinuous.applyTo(rightSide);
                blueSolidPattern.applyTo(centerSide);
                blueSolidPattern.applyTo(underShooter);
                System.out.println("BLUE");
                isRedTeam = false;
            } else {
                rainbowScroll.applyTo(m_ledBuffer);
                isRedTeam = false;
            }
        }
    }

    /** Set colors on top and under-shooter LEDs to a color based on alliance color. 
     * 
     * @param redPattern Red-colored pattern to set
     * @param bluePattern Blue-colored pattern to set
     */
    private void setTopLEDTeamColor(LEDPattern redPattern, LEDPattern bluePattern){
        if(isRedTeam){
                redPattern.applyTo(centerSide);
                redPattern.applyTo(underShooter);
                System.out.println("RED");
        } else {
            bluePattern.applyTo(centerSide);
            bluePattern.applyTo(underShooter);
            System.out.println("BLUE");
        } 
    }
    

    /** Set colors on side LEDs to a color based on alliance color. 
     * 
     * @param redPattern Red-colored pattern to set
     * @param bluePattern Blue-colored pattern to set
     */
    private void setSideLEDTeamColor(LEDPattern redPattern, LEDPattern bluePattern){
        if(isRedTeam){
            redPattern.applyTo(leftSide);
            redPattern.applyTo(rightSide);
            System.out.println("RED");
        } else {
            bluePattern.applyTo(leftSide);
            bluePattern.applyTo(rightSide);
            System.out.println("BLUE");
        } 
    }


    /** Sets non-drivetrain motors to speed 0.
     * 
     * @return A Command
     */
    public Command stopMotors(){
        return run(() -> {
            intakeSpin.set(0); 
            intakeDownL.set(0); 
            intakeDownR.set(0); 
            shooterFlywheel.set(0); 
            shooterIn.set(0); 
        });
    }
   

    /*
    //Original attempt at centering on an AprilTag (we went with different code in the end though)
    public Command centerOnAprilTag(SwerveDrive drivetrain, Translation2d position, double rotSpeed, double range){
        return run(() -> {
            double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
            System.out.println("CENTER ON APRIL");
            if(Math.abs(tx) > range){
                if(tx > 0){
                    drivetrain.drive(position, -rotSpeed * (tx/range), false, false);
                } else {
                    System.out.println("left");
                    drivetrain.drive(position, rotSpeed * (tx/range), false, false);
                }
                    //rotSpeed * (tx/range)
            }

        });
    }
    */


    public Command runIntake(double speed){
      return run( () -> {
        intakeSpin.set(speed);
      });
    }

    public Command runIntakeWithJoystick(Joystick joystick){
      return run( () -> {
        intakeSpin.set(-1 * (joystick.getY() * Constants.intakeSpinMaxSpeed));
      });
    }


    // double autoElevationSpeed = Constants.elevatorSpeed / (1 - Math.abs(height) / goal);
    /** Moves intake with the flippy thingy on the joystick (Z-axis in the code). 
     * 
     * @param controller CommandJoystick controller to use as input
     * @return A Command
     */
    public Command moveIntakeWithFlippyThingy(CommandJoystick controller){
        return run( () -> {
            double goal = Math.abs(-1.0 - controller.getZ())/2.0 * Constants.intakeMoveRange;
            double difference = Math.abs(intakeInitialPosition - intakeDownL.getEncoder().getPosition());
            double speed = 0;

            if(Math.abs(goal - difference) > Constants.intakeAngleThreshHold){
                speed = Constants.intakeMoveMaxSpeed * ((goal - difference)/Math.abs(goal - difference)); //take just the sign of goal-difference (fastest way we thought of)
                //without the multiplying by stuff it would try to go down even more when the setpoint (goal) was moved up to try to tell the intake to go back up, so the intake would try to go down when it needed to go up
            } else {
                speed = Constants.intakeMoveMaxSpeed * ((goal - difference)/Constants.intakeAngleThreshHold);
            }

            //Math.abs(-1.0 - )/2.0 * Constants.intakeMoveRange)
            intakeDownL.set(speed);
            intakeDownR.set(-speed);
           
            //Print stuff for troubleshooting
            // System.out.println("intakeDownL Encoder Position: " + intakeDownL.getEncoder().getPosition());
            // System.out.println("Goal: " + goal);
            // System.out.println("Initial position: " + intakeInitialPosition);
            // System.out.println("Difference: " + difference);
            // System.out.println("Speed to set: " + speed);
        });
    }

    public Command intakeDownAutonomous(){
        return runOnce( () -> {
            
            //double goal = Constants.intakeMoveRange;
            double goal = 10;
            double difference = Math.abs(intakeInitialPosition - intakeDownL.getEncoder().getPosition());
            double speed = 0;
            System.out.println("iuhafkgdfjhfe");
            while(Math.abs(goal - difference) > 0.5){
                System.out.println("AAAAA");
                difference = Math.abs(intakeInitialPosition - intakeDownL.getEncoder().getPosition());
                System.out.println("Difference: " + difference);
                // if(Math.abs(goal - difference) > Constants.intakeAngleThreshHold){
                //     speed = (Constants.intakeMoveMaxSpeed * ((goal - difference)/Math.abs(goal - difference)))/2; //take just the sign of goal-difference (fastest way we thought of)
                //     //without the multiplying by stuff it would try to go down even more when the setpoint (goal) was moved up to try to tell the intake to go back up, so the intake would try to go down when it needed to go up
                // } else {
                //     speed = (Constants.intakeMoveMaxSpeed * ((goal - difference)/Constants.intakeAngleThreshHold))/2;
                // }
                speed = 0.15;
                System.out.println("Speed to set: " + speed);
                //Math.abs(-1.0 - )/2.0 * Constants.intakeMoveRange)
                intakeDownL.set(speed);
                intakeDownR.set(-speed);
            }
        });
    }

    public Command moveIntakeWithButtons(double speed){
        return run( () -> {
        
            if(((speed > 0) && (intakeDownL.getEncoder().getPosition() >= intakeInitialPosition - 2)) || ((speed <= 0) && (intakeDownL.getEncoder().getPosition() <= (Constants.intakeMoveRange + intakeInitialPosition)))) {
                intakeDownL.set(-speed);
                intakeDownR.set(speed);
            }
            
            //Print stuff for troubleshooting
            // System.out.println("intakeDownL Encoder Position: " + intakeDownL.getEncoder().getPosition());
            // System.out.println("Speed to set: " + speed);
        });
    }


    public Command chargeShooterFlywheel(double speed){
        return run( () -> {
            shooterFlywheel.set(speed);
        });
    }


    /** Manually charge shooter flywheel using input from joystick
     * 
     * @param joystick CommandJoystick to use as input
     * @return A Command
     */
    public Command chargeShooterFlywheelWithJoystick(CommandJoystick joystick){
        return run( () -> {
            shooterFlywheel.set(-1 * (joystick.getY() * Constants.launcherMaxSpeed));
            
            //System.out.println("Speed to set: " + -1 * thing.getY() * Constants.launcherMaxSpeed);
            //System.out.println("Velocity of motor: " + shooterFlywheel.getEncoder().getVelocity() + " RPM");
        });
    }


    /**
     * 
     * @param speed Speed (between -1 and 1) to set the shooter's intake wheel motor to
      * @return A Command
     * 
     */
    public Command setShooterIn(double speed){
        //TUNE SPEED LATER!!
        return run( () -> {
            shooterIn.set(speed);
            //System.out.println(speed);
        });
    }

    public Command slowDrive(XboxController controller, SwerveDrive swerve){
        return run (() -> {
            Translation2d translation = new Translation2d( controller.getLeftY() * -1, controller.getLeftX() * -1); 
            swerve.drive(translation, controller.getRightX(), false, false);
            System.out.println("SLLLLLLLOOOOOOWWWWWWWWWWWWW MOOOOOOOODDDDDDEEEEE");           
        });
    }

    public Command fireShooterManual(Joystick joystick){
        return run( () -> {
            double passSpeed = 0.7;
            double suckerSpeed = 0.65;
            shooterFlywheel.set(passSpeed);

            System.out.println("Launcher Velocity: " + shooterFlywheel.getEncoder().getVelocity());
            if(shooterFlywheel.getEncoder().getVelocity() < passSpeed * 6000 - 1000){
                shooterIn.set(0);
            }else{
                shooterIn.set(suckerSpeed);
            }
            // if(joystick.getRawButton(5)){
            //     shooterFlywheel.set(0.8);
            //     System.out.println("Flywheel spinning");
            //     //-1 * (joystick.getY() * Constants.launcherMaxSpeed)
            // } //else {
            //     shooterFlywheel.set(Constants.launcherIdleSpeed);
            // }
            // if(joystick.getRawButton(1) && shooterIn.getEncoder().getVelocity() >= (0.8 * Constants.velocitySlope)){ //&& shooterIn.getEncoder().getVelocity() > 1000
            //     shooterIn.set(0.65);
            // } else {
            //     shooterIn.set(0);
            // }
        });
    }

    public Command fireShooterAutonomous(){
        return runOnce( () -> {
            int clockTimerMax = 500;
            //boolean hasTarget = LimelightHelpers.getTV("");
            while(clockTimer < clockTimerMax){
                clockTimer++;
                double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
                    
                double distance = (Constants.hubAprilTagHeight - Constants.limelightHeight) / (Math.tan(Math.toRadians(Constants.limelightPitch + ty))); //Estimated distance from detected AprilTag
                    
                double goalSpeed = distance/(Constants.launcherMaxRange/(Constants.launcherMaxSpeed - Constants.launcherMinSpeed)) + Constants.launcherMinSpeed;
                
                if(distance > Constants.launcherMaxRange){
                    goalSpeed = Constants.launcherMaxSpeed;
                }

                if(distance <= 0){
                    goalSpeed = Constants.launcherMinSpeed;
                }
                
                shooterFlywheel.set(goalSpeed);

                double currentVelocity = shooterFlywheel.getEncoder().getVelocity(); //may change later 
                    
                double lowestAcceptableVelocity = Constants.velocitySlope * goalSpeed + Constants.launcherMinRPM;

                //if actual velocity isn't close enough then pause small wheels and wait for flywheel to speed up
                if(currentVelocity < lowestAcceptableVelocity){
                    shooterIn.set(0);
                } else {
                    shooterIn.set(0.65);
                }
        }});
    }

    /** Spins the launcher flywheel according to estimated distance from detected AprilTag, adjusting the speed to score effectively.
     *  
     * @return A Command
     */
    public Command fireShooterAutomatic(Joystick joystick){
        return run( () -> {
            boolean hasTarget = LimelightHelpers.getTV("");
            if(hasTarget &&  !joystick.getRawButton(5)){
                double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
                    
                double distance = (Constants.hubAprilTagHeight - Constants.limelightHeight) / (Math.tan(Math.toRadians(Constants.limelightPitch + ty))); //Estimated distance from detected AprilTag
                    
                double goalSpeed = distance/(Constants.launcherMaxRange/(Constants.launcherMaxSpeed - Constants.launcherMinSpeed)) + Constants.launcherMinSpeed;
                
                if(distance > Constants.launcherMaxRange){
                    goalSpeed = Constants.launcherMaxSpeed;
                }

                if(distance <= 0){
                    goalSpeed = Constants.launcherMinSpeed;
                }
                
                shooterFlywheel.set(goalSpeed);

                double currentVelocity = shooterFlywheel.getEncoder().getVelocity(); //may change later 
                    
                double lowestAcceptableVelocity = Constants.velocitySlope * goalSpeed + Constants.launcherMinRPM;

                //if actual velocity isn't close enough then pause small wheels and wait for flywheel to speed up
                // if(currentVelocity < lowestAcceptableVelocity){
                //     shooterIn.set(0);
                // } else {
                //     shooterIn.set(0.65);
                // }
                if(currentVelocity >= lowestAcceptableVelocity){
                    shooterIn.set(0.65);
                } else {
                    shooterIn.set(0);
                }
                //10, 26

                // System.out.println("Calculated distance: " + distance);
                // System.out.println("tY: " + ty);
                // System.out.println("goalSpeed: " + goalSpeed);

                //Stuff for figuring out equations and code
                //
                //0.55 ~~ 3200 RPM (intake touching hub) (0.6604 m)
                //0.65 ~~ 
                //0.75 ~~ 
                //0.85 ~~ 4850 RPM (2.9m from limelight)
                //ranges 
                //0.3 
                //1750 RPM
                //2.24 m
                //
                
            }// else {
            //     if(joystick.getRawButton(5)){
            //         shooterFlywheel.set(0.8);
            //         //-1 * (joystick.getY() * Constants.launcherMaxSpeed)
            //     } else {
            //         shooterFlywheel.set(Constants.launcherIdleSpeed);
            //     }
            //     if(joystick.getRawButton(1)){
            //         shooterIn.set(0.65);
            //     } else {
            //         shooterIn.set(0);
            //     }
            // }
        });
    }

    public Command centerOnAprilTag(XboxController controller, SwerveDrive swerve){
        return run(() -> {
                Translation2d translation = new Translation2d( controller.getLeftY() * -1, controller.getLeftX() * -1); 
                swerve.drive(translation, LimelightHelpers.getTX("") * -0.05, false, false);                
        });
    }
    int centerLimelightTimer = 0;
    public Command centerOnAprilTagAuto(SwerveDrive swerve){
        return runOnce(() -> {
            centerLimelightTimer = 0;
            autoTX = LimelightHelpers.getTX("");
            while(autoTX == 0  && centerLimelightTimer < 1000){
                centerLimelightTimer++;
                autoTX = LimelightHelpers.getTX("");
            }

            while(Math.abs(autoTX) > 1){
                if(LimelightHelpers.getTX("") == 0){
                    autoTX = LimelightHelpers.getTX("");
                }
                Translation2d translation = new Translation2d( 0, 0); 
                swerve.drive(translation, LimelightHelpers.getTX("") * -0.05, false, false);                
                System.out.println("Centering!!");
            }
            System.out.println("Done!");
        });
    }

}
