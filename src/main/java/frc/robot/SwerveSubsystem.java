package frc.robot;
//.subsystems.swervedrive

import static edu.wpi.first.units.Units.Meter;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

//A lot of the stuff in this class is either pathplanner or making some of the SwerveDrive methods usable with this
//That's probably why the subsystem class in YAGSL example is so huge
public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveDrive  swerveDrive;

    //unrelated(ish) but i'm glad I took APCS it's a fun class!! if you're reading this and haven't taken it and like programming u should take AP comp sci :) (2nd semester especially gets pretty interesting)

    public SwerveSubsystem(File swerveJsonDirectory) {

        boolean blueAlliance = false;
        //startingPose copied from YAGSL example, it basically says if you're on the blue alliance then make the pose one thing and if not then do another thing
        //? and : are basically an if/else if i understand this correctly
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));

        try { //don't remove this try/catch (code needs to know what to do if it can't find one of the JSON files it needs)
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.maxSpeed,startingPose);
            //i'm missing a json file but WHAT FILE IS MISSING AAAAAAAAAAAAAAAAAAAAAAAA
            //so it turns out a file wasn't missing, i just didn't think to put this into a constructor at first...
        } catch(IOException e){
            throw new RuntimeException(e);
            //learn what runtime exception means
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        

        //Set individual PIDs for modules here if needed, shouldn't be necessary tho (try inverting drive motors, IMU, and maybe angle motors in JSON files first)
        //0-3 FL to BL clockwise
        //Starting point: P = 0.0135, I = 0, D = 0.3
        /*
        swerveDrive.getModules()[0].setAnglePIDF(new PIDFConfig(0.0135,0,0,0)); //FL
        swerveDrive.getModules()[1].setAnglePIDF(new PIDFConfig(0,0,0,0)); //FR
        swerveDrive.getModules()[2].setAnglePIDF(new PIDFConfig(0,0,0,0)); //BL
        swerveDrive.getModules()[3].setAnglePIDF(new PIDFConfig(0,0,0,0)); //BR
        */
        
        setupPathPlanner();
    }

    public void setupPathPlanner(){
      RobotConfig config;

      try{
        config = RobotConfig.fromGUISettings();
        final boolean enableFeedForward=true;
        AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedForward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
             new PIDConstants(5.0, 0.0, 0.0),
             new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
        );
      }catch(Exception e){
        e.printStackTrace();
      }
      PathfindingCommand.warmupCommand().schedule();
    }

    public SwerveDrive getSwerveDrive(){
      return swerveDrive;
    }

    public String printEncoder(int module){
      double angleMotor = this.getSwerveDrive().getModules()[module].getAngleMotor().getPosition(); // getAbsoluteEncoder().getAbsolutePosition();
      double driveMotor = this.getSwerveDrive().getModules()[module].getDriveMotor().getPosition();
      double encoder = this.getSwerveDrive().getModules()[module].getAbsoluteEncoder().getAbsolutePosition();

      return "Angle Motor: " + angleMotor + "\nDrive Motor: " + driveMotor + "\nEncoder: " + encoder + "\n- - - - - - - - - - - - - -";
    }
    
    //For SmartDashboard (ignore for next year, somewhere said that shuffleboard is gonna be removed in 2027 season or something)
    // public void initSendable(SendableBuilder builder){
    //     builder.setSmartDashboardType("Encoders");

    //     double frontLeftEncoder = this.getSwerveDrive().getModules()[0].getAbsoluteEncoder().getAbsolutePosition();
    //     double frontRightEncoder = this.getSwerveDrive().getModules()[1].getAbsoluteEncoder().getAbsolutePosition();
    //     double backLeftEncoder = this.getSwerveDrive().getModules()[2].getAbsoluteEncoder().getAbsolutePosition();
    //     double backRightEncoder = this.getSwerveDrive().getModules()[3].getAbsoluteEncoder().getAbsolutePosition();
        
    //     builder.addDoubleProperty("FrontLeft",frontLeftEncoder,null);
    // }
    
    /**
    * Drive the robot given a chassis field oriented velocity.
    *
    * @param velocity Velocity according to the field.
    */
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
    {
      return run(() -> {
        swerveDrive.driveFieldOriented(velocity.get()); 
      });
    }

    
  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }
  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }
  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
   
    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
      return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose){
      swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity(){
      return swerveDrive.getRobotVelocity();
    }

    

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
      swerveDrive.zeroGyro();
      
    }

    /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drivebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance())  {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else  {
      zeroGyro();
    }
  }
    // public void centerOnAprilTag(){

    // }

}