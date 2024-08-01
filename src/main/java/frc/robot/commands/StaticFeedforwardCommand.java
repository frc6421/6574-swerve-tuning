// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StaticFeedforwardCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final double voltageDelta;
  private double setVoltage;

  /**
   * Distance used to determine if the robot is moving.
   * </p>
   * *** May want to change value depending on data output.
   */
  private final static double ROBOT_IS_MOVING_METERS = 0.01;

  /**
   * Used to determine the feedforward voltage needed to just get the robot to
   * move.
   * </p>
   * When done update the kS value of the Drive Motor
   * {@link DriveSubsystem#DRIVE_GAINS}.
   * </p>
   * *** May need to be updated if the weight of the robot changes significantly.
   * 
   * @param drive the drive subsystem used with command.
   */
  public StaticFeedforwardCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // The change in velocity to add each robot cycle.
    voltageDelta = 0.001;

    Shuffleboard.getTab("3: Static Feedforward").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set voltage to zero each time the command starts
    setVoltage = 0;
    // Point wheels forward before resetting odometry.  This methods should do this
    driveSubsystem.setVelocity(0);
    // Reset odometry
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forward at set speed
    driveSubsystem.setDriveStraightVoltage(setVoltage);
    // Increase chassis speed.
    setVoltage += voltageDelta;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Print the voltage of each drive motor module.
    System.out.println("Final set voltage: " + setVoltage);
    for (int i = 0; i < 4; ++i) {
      System.out.println("FInal Static Feedforward Voltage Module " + i + ": "
          + driveSubsystem.getMaxSwerveModuleVoltage()[i]);
      System.out.println("Final Static Feedforward Output Module " + i + ": "
          + driveSubsystem.getMaxSwerveModuleOutput()[i]);
    }
    // Stop the robot.
    driveSubsystem.setDriveStraightVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true of robot moves 1 cm
    return driveSubsystem.getPose().getX() >= ROBOT_IS_MOVING_METERS;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity (meters/sec)", () -> driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond, null);
    builder.addDoubleProperty("Module 0 Voltage",
        () -> driveSubsystem.getMaxSwerveModuleVoltage()[0], null);
    builder.addDoubleProperty("Module 0 Output",
        () -> driveSubsystem.getMaxSwerveModuleOutput()[0], null);
    builder.addDoubleProperty("Module 1 Voltage",
        () -> driveSubsystem.getMaxSwerveModuleVoltage()[1], null);
    builder.addDoubleProperty("Module 1 Output",
        () -> driveSubsystem.getMaxSwerveModuleOutput()[1], null);
    builder.addDoubleProperty("Module 2 Voltage",
        () -> driveSubsystem.getMaxSwerveModuleVoltage()[2], null);
    builder.addDoubleProperty("Module 2 Output",
        () -> driveSubsystem.getMaxSwerveModuleOutput()[2], null);
    builder.addDoubleProperty("Module 3 Voltage",
        () -> driveSubsystem.getMaxSwerveModuleVoltage()[3], null);
    builder.addDoubleProperty("Module 3 Output",
        () -> driveSubsystem.getMaxSwerveModuleOutput()[3], null);
  }

}
