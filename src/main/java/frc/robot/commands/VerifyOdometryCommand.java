// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class VerifyOdometryCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  /**
   * Verify that odometry calcualtion is same as actual distance travelled.
   * </p>
   * If odometry is off by more than a acceptable range for the team, it is
   * suggested that the wheel be physically measured with a caliper and the
   * measured value entered into {@link DriveSubsystem#WHEEL_RADIUS_INCHES}.
   * </p>
   * After the value is changed the command should be run again to verify the
   * odometry is within the acceptable range.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VerifyOdometryCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // Add command to shuffleboard.
    Shuffleboard.getTab("1: Verify Odometry").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set drive motors to coast
    driveSubsystem.setDriveIdleMode(IdleMode.kCoast);

    // Point wheels forward
    // driveSubsystem.setDriveStraightVoltage(0);
    driveSubsystem.setVelocity(0);

    // Reset robot Pose
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do nothing. Manually move the robot to distances.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Final robot distance (meters) :" + driveSubsystem.getPose().getX());
    for (int i = 0; i < 4; ++i) {
      System.out.println(
          "Module " + i + " distance (meters): " + driveSubsystem.getModulePositions()[i].distanceMeters);
    }
    // Set drive motors to brake.
    driveSubsystem.setDriveIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Robot Distance (meters)", () -> driveSubsystem.getPose().getX(), null);
    builder.addDoubleProperty("Module 0 Distance (meters)",
        () -> driveSubsystem.getModulePositions()[0].distanceMeters, null);
    builder.addDoubleProperty("Module 1 Distance (meters)",
        () -> driveSubsystem.getModulePositions()[1].distanceMeters, null);
    builder.addDoubleProperty("Module 2 Distance (meters)",
        () -> driveSubsystem.getModulePositions()[2].distanceMeters, null);
    builder.addDoubleProperty("Module 3 Distance (meters)",
        () -> driveSubsystem.getModulePositions()[3].distanceMeters, null);
  }
}
