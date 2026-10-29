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

  //autonomous selection (pretty sure it's for pathplanner)
  private final SendableChooser<Command> autoChooser;

  //Two made for each because command controllers have no getButton methods but regular controllers do (needed for custom Triggers and if some of the commands need a button press without scheduling another command)
  CommandXboxController drivetrainController = new CommandXboxController(0);
  XboxController controller = new XboxController(0);
  
  //WOOHOOOO SWERVE DRIVE FINALLY SET UP
  SwerveSubsystem swervy = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  
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

  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("testCommand", Commands.print("I EXIST"));
    autoChooser = AutoBuilder.buildAutoChooser("Test");
  }

  private void configureBindings() {
    
    Command driveFieldOrientedDirectAngle       =  swervy.driveFieldOriented(driveDirectAngle); //Makes robot point towards right joystick direction
    Command driveFieldOrientedAngularVelocity   =  swervy.driveFieldOriented(driveAngularVelocity); //Makes robot rotate using right joystick x-axis
    Command driveRobotOriented                  =  swervy.driveFieldOriented(driveRobotOrientedInputStream); 

    //Left joystick controls translation, right controls rotation but has different settings
    swervy.setDefaultCommand(driveFieldOrientedAngularVelocity); //direct angle is default (point to joystick direction)
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();   
  }

  
}
