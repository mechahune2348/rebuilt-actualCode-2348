// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
 
  private final RobotContainer m_robotContainer;
  // private final XboxController c1;
  // private final TalonFX krakenmotor;
  //private final SparkMax JPP;
  //private final SparkMax sparky;
  // double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
  // double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
  // double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
  // boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

  public Robot() {
    m_robotContainer = new RobotContainer();
    // c1 = new XboxController(0);
    // krakenmotor = new TalonFX(53);
    //JPP = new SparkMax(8,MotorType.kBrushless);
    //sparky = new SparkMax(7,MotorType.kBrushless);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //m_robotContainer.printEncoders();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    if(RobotContainer.mechanismsNonCommand.getRawButtonReleased(1) || RobotContainer.mechanismsNonCommand.getRawButtonReleased(5)){
      CommandSubsystem.shooterIn.set(0);
    }
    //krakenmotor.set(c1.getLeftY()/2);
    
    // System.out.println(c1.getLeftY());

    //System.out.println("Y: " + m_robotContainer.mechanismsController.getY());

    // JPP.set(c1.getLeftY()/1.5);
    // sparky.set(c1.getRightY());
    //System.out.println("Intake Up/Down Encoder Position: " + m_robotContainer.robotCommands.intakeDownL.getEncoder().getPosition());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
