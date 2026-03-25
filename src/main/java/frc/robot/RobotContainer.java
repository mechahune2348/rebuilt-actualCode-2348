// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Joystick; 


public class RobotContainer {
  //these are mostly used within the command methods so that they actually change but keeping them here for copy/pasting faster
  double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
  double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
  double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
  boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

  //autonomous selection (pretty sure it's for pathplanner)
  private final SendableChooser<Command> autoChooser;

  //Two made for each because command controllers have no getButton methods but regular controllers do (needed for custom Triggers and if some of the commands need a button press without scheduling another command)
  CommandXboxController drivetrainController = new CommandXboxController(0);
  XboxController controller = new XboxController(0);

  CommandJoystick mechanismsController = new CommandJoystick(1);
  static Joystick mechanismsNonCommand = new Joystick(1);
  
  //WOOHOOOO SWERVE DRIVE FINALLY SET UP
  SwerveSubsystem swervy = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  CommandSubsystem robotCommands = new CommandSubsystem();
  
  //inputstreams copied from example
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swervy.getSwerveDrive(), () -> drivetrainController.getLeftY() * -1, () -> drivetrainController.getLeftX() * -1)
                                                            .withControllerRotationAxis(drivetrainController::getRightX) 
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(drivetrainController::getRightX,
                                                                                             drivetrainController::getRightY)
                                                           .headingWhile(true);                                                        

    /**
   * Clones the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOrientedInputStream = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);                                                         


  // DoubleSupplier limelightX = () -> tx;

  // DoubleSupplier limelightY = () -> {
  //   return Math.sqrt( Math.pow(ta,2) - Math.pow(tx,2) );
  // };

  // PIDController aaa = new PIDController(0,0,0);

  // //controller range is -1 to 1 so figure out how to get from percentage (tA) to that range and from pixels (tX) to that range
  // //maybe have a pixel range and ends of that range are -1 and 1 and center is center
  // //
  // SwerveInputStream limelightDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(limelightX,
  //                                                                                            limelightY)
  //                                                          .headingWhile(true).robotRelative(true)
  //                                                            .allianceRelativeControl(false);          

  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("testCommand", Commands.print("I EXIST"));
    NamedCommands.registerCommand("fireShooter", robotCommands.fireShooterAutonomous());
    NamedCommands.registerCommand("centerOnAprilTag", robotCommands.centerOnAprilTagAuto(swervy.getSwerveDrive()));
    NamedCommands.registerCommand("intakeDown", robotCommands.intakeDownAutonomous());

    
    autoChooser = AutoBuilder.buildAutoChooser("Test");
    //autoChooser.setDefaultOption("Do Nothing", Commands.none());
       
    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", swervy.driveForward().withTimeout(1));

    autoChooser.addOption("Do Nothing", Commands.none());
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // double frontLeftEncoder = swervy.getSwerveDrive().getModules()[0].getAbsoluteEncoder().getAbsolutePosition();
    // double frontRightEncoder =swervy.getSwerveDrive().getModules()[1].getAbsoluteEncoder().getAbsolutePosition();
    // double backLeftEncoder = swervy.getSwerveDrive().getModules()[2].getAbsoluteEncoder().getAbsolutePosition();
    // double backRightEncoder = swervy.getSwerveDrive().getModules()[3].getAbsoluteEncoder().getAbsolutePosition();
    //SmartDashboard.putData();
  }
  
  public void printEncoders(){
    System.out.println("Front Left\n" + swervy.printEncoder(0)); //front left
    System.out.println("Front Right\n" + swervy.printEncoder(1)); //front right
    System.out.println("Back Left\n" + swervy.printEncoder(2)); //back left
    System.out.println("Back Right\n" + swervy.printEncoder(3)); //back right
  }

  private void configureBindings() {
    
    Command driveFieldOrientedDirectAngle       =  swervy.driveFieldOriented(driveDirectAngle); //Makes robot point towards right joystick direction
    Command driveFieldOrientedAngularVelocity   =  swervy.driveFieldOriented(driveAngularVelocity); //Makes robot rotate using right joystick x-axis
    Command driveRobotOriented                  =  swervy.driveFieldOriented(driveRobotOrientedInputStream); 

    //Drivetrain Controller Stuff
    //Left joystick controls translation, right controls rotation but has different settings
    swervy.setDefaultCommand(driveFieldOrientedAngularVelocity); //direct angle is default (point to joystick direction)
    drivetrainController.leftBumper().whileTrue(driveFieldOrientedDirectAngle); //left or right makes robot spin in that direction
    drivetrainController.rightBumper().whileTrue(driveRobotOriented); //forward moves in direction robot is facing
    
    drivetrainController.x().onTrue(Commands.runOnce(swervy::zeroGyro));
    drivetrainController.b().whileTrue(robotCommands.centerOnAprilTag(controller, swervy.getSwerveDrive()).repeatedly());
    drivetrainController.a().whileTrue(robotCommands.slowDrive(controller, swervy.getSwerveDrive()).repeatedly());
    
    //Mechanisms Controller Stuff 

    //BooleanSuppliers made for custom trigger events, mostly for stopping motors but also for switching between automatic and manual firing of the ball launcher
    
    // BooleanSupplier intakeWheelButtonsPressed = () -> {
    //     return mechanismsNonCommand.getRawButton(3) || mechanismsNonCommand.getRawButton(2);
    // };
    // Trigger intakeSpinning = new Trigger(intakeWheelButtonsPressed);


    BooleanSupplier intakeButtonsPressed = () -> {
        return mechanismsNonCommand.getRawButton(6) || mechanismsNonCommand.getRawButton(7);
    };
    Trigger intakeMoving = new Trigger(intakeButtonsPressed);


    BooleanSupplier useAutomaticShooter = () -> {
      // (LimelightHelpers.getFiducialID("") == 10 || LimelightHelpers.getFiducialID("") == 26) &&
        return hasTarget && !mechanismsNonCommand.getRawButton(5);
    }; 
    //apriltagsdetectedwahahauwhdasjhfdosomethingfocusthinkpleaseee
    //!hasTarget or button 5 pressed?
    //hasTarget and button5 not pressed?
    

    BooleanSupplier shooterShootingButtons = () -> {
        return mechanismsNonCommand.getRawButton(1) && !mechanismsNonCommand.getRawButton(5);
    };
    Trigger shooterFiringAutomatic = new Trigger(shooterShootingButtons);

    BooleanSupplier bothShooterShootingButtons = () -> {
        return mechanismsNonCommand.getRawButton(1) || mechanismsNonCommand.getRawButton(5);
    };
    Trigger shooterFiring = new Trigger(bothShooterShootingButtons);

    robotCommands.setDefaultCommand(robotCommands.stopMotors());

    mechanismsController.button(4).whileTrue(robotCommands.chargeShooterFlywheel(-1.5));
    //mechanismsController.button(4).whileTrue(robotCommands.setShooterIn(-1.5));
    //mechanismsController.button(2).whileFalse(robotCommands.chargeShooterFlywheel(0).andThen(robotCommands.shootBall(0)));
    
    // mechanismsController.button(3).whileTrue(robotCommands.runIntake(0.2));
    // mechanismsController.button(2).whileTrue(robotCommands.runIntake(-0.2));
    // intakeSpinning.whileFalse(robotCommands.runIntake(0));
    mechanismsController.axisMagnitudeGreaterThan(1, 0.05).whileTrue(robotCommands.runIntakeWithJoystick(mechanismsNonCommand));
    
    //mechanismsController.button(1).and(useAutomaticShooter).whileTrue(robotCommands.fireShooterAutomatic().repeatedly());
    mechanismsController.button(5).whileTrue(robotCommands.fireShooterManual(mechanismsNonCommand));
    //mechanismsController.button(5).whileFalse(robotCommands.fireShooterAutomatic(mechanismsNonCommand));
    shooterFiringAutomatic.whileTrue(robotCommands.fireShooterAutomatic(mechanismsNonCommand));
    //mechanismsController.button(1).whileFalse(robotCommands.setShooterIn(0));
    shooterFiring.whileFalse(robotCommands.chargeShooterFlywheel(Constants.launcherIdleSpeed));

    mechanismsController.button(6).whileTrue(robotCommands.moveIntakeWithButtons(0.1)); 
    mechanismsController.button(7).whileTrue(robotCommands.moveIntakeWithButtons(-1 * 0.1));
    intakeMoving.whileFalse(robotCommands.moveIntakeWithButtons(0));//stop intake motors if neither button is pressed


    /* Keybinds
    * 
    * Drivetrain (XboxController):
    * Left joystick - translation
    * Right joystick - rotation
    * Left bumper (hold) - switch rotation mode to direct angle
    * Right bumper (hold) - switch driving mode to robot-oriented
    * X button - zero gyro (field-oriented forward becomes where back of robot faces)
    * B button - center on AprilTag
    * 
    * Scoring Mechanisms (Joystick):
    * Trigger (button 1) - Shoot balls (also controls shooter flywheel if automatic firing mode)
    * Joystick (Y-axis) - Run intake
    * Button 2 -     -
    * Button 3 -     -
    * Button 4 - Reverse shooter motors (to unjam)
    * Button 5 - Hold down to switch to manual shooting and charge shooter flywheel
    * Button 6 - Move intake up
    * Button 7 - Move intake down
    * 
    */

    
    /*
    //Old keybinds
    BooleanSupplier intakeButtonsPressed = () -> {
        return mechanismsNonCommand.getRawButton(6) || mechanismsNonCommand.getRawButton(7);
    };

    Trigger intakeMoving = new Trigger(intakeButtonsPressed);


    BooleanSupplier flywheelButtonsPressed = () -> {
        return mechanismsNonCommand.getRawButton(3) || mechanismsNonCommand.getRawButton(2);
    };

    Trigger flywheelMoving = new Trigger(flywheelButtonsPressed);


    BooleanSupplier shooterButtonsPressed = () -> {
        return mechanismsNonCommand.getRawButton(4) || mechanismsNonCommand.getRawButton(5);
    };

    Trigger shooterSpinning = new Trigger(shooterButtonsPressed);

    mechanismsController.button(6).whileTrue(robotCommands.moveIntakeWithButtons(0.1)); 
    mechanismsController.button(7).whileTrue(robotCommands.moveIntakeWithButtons(-1 * 0.1));
    intakeMoving.whileFalse(robotCommands.moveIntakeWithButtons(0));//stop intake motors if neither button is pressed

    mechanismsController.button(4).whileTrue(robotCommands.shootBall(0.65));
    mechanismsController.button(5).whileTrue(robotCommands.shootBall(-0.5));
    shooterSpinning.whileFalse(robotCommands.shootBall(0));   

    mechanismsController.button(1).whileTrue(robotCommands.fireShooter().repeatedly());
    mechanismsController.button(1).whileFalse(robotCommands.chargeShooterFlywheel(0));

    //mechanismsController.axisMagnitudeGreaterThan(1,0).whileTrue(robotCommands.chargeIntakeFlywheel(mechanismsController));

    mechanismsController.button(3).whileTrue(robotCommands.runIntake(0.4));
    mechanismsController.button(3).whileFalse(robotCommands.runIntake(0)); 

    */

    //For CommandJoystick 
    //Axis 0 = X (big joystick side-to-side) (not really used this year)
    //Axis 1 = Y (big joystick front-to-back)
    //Axis 2 = little flippy thingy (thing at bottom with + at top and - at bottom)
    //Other buttons are numbered on physical controller
    //Trigger on big joystick is button 1

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();   
  }

  
}
