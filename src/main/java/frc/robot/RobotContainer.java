// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StaticFeedforwardCommand;
import frc.robot.commands.TuneRotationPCommand;
import frc.robot.commands.TuneVelocityPCommand;
import frc.robot.commands.VelocityFeedforwardCommand;
import frc.robot.commands.VerifyOdometryCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems \\
  private final DriveSubsystem driveSubsystem;

  // Commands \\
  private final VerifyOdometryCommand verifyOdometryCommand;
  private final StaticFeedforwardCommand staticFeedforwardCommand;
  private final VelocityFeedforwardCommand velocityFeedforwardCommand;
  private final TuneVelocityPCommand tuneVelocityPCommand;
  private final TuneRotationPCommand tuneRotationPCommand;

  //Controllers \\  
  private final XboxController driverController;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems \\
    driveSubsystem = new DriveSubsystem();
    
    // Commands \\
    verifyOdometryCommand = new VerifyOdometryCommand(driveSubsystem);
    staticFeedforwardCommand = new StaticFeedforwardCommand(driveSubsystem);
    velocityFeedforwardCommand = new VelocityFeedforwardCommand(driveSubsystem);
    tuneVelocityPCommand = new TuneVelocityPCommand(driveSubsystem);
    tuneRotationPCommand = new TuneRotationPCommand(driveSubsystem);

    // Controllers \\
    driverController = new XboxController(0);

    // Configure default commands
    driveSubsystem.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> driveSubsystem.drive(
              -MathUtil.applyDeadband(driverController.getLeftY(), 0.05),
              -MathUtil.applyDeadband(driverController.getLeftX(), 0.05),
              -MathUtil.applyDeadband(driverController.getRightX(), 0.05),
              true, true),
          driveSubsystem));


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
