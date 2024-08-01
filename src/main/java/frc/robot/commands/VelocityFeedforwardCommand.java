// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class VelocityFeedforwardCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  /**
   * The velocity to drive the robot in this command.
   * </p>
   * Can be positive or negative but should be less than max robot velocity
   */
  private double voltage;
  
  // Used to display the calculated kV values on the Shuffleboard
  private double[] modulekV = {0.0,0.0,0.0,0.0};

  /**
   * Used to determine the feed forward voltage needed to get the robot moving
   * close to the set velocity.
   * </p>
   * This should be run at several different velocities, both froward and
   * backward, to verify that the data appears good. kV should be fairly linear
   * over the range of velocities. These velocities are drive motor velocity in
   * rpm. (?)
   * </p>
   * When done update the {@link DriveSubsystem#kV} value of the Drive Motor
   * {@link DriveSubsystem#DRIVE_GAINS}.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VelocityFeedforwardCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    voltage = 0; 

    // Swerve request to make sure the wheels point in the x direction
    // Swerve request to use to drive the robot
    // Swerve request to stop the robot

    Shuffleboard.getTab("4: Velocity Feedforward").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set wheels to face forward.
    driveSubsystem.setDriveStraightVoltage(0);
    
    // Reset odometry
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setDriveStraightVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < 4; ++i) {
      double voltage = driveSubsystem.getMaxSwerveModuleVoltage()[i];
      double velocity = driveSubsystem.getModuleStates()[i].speedMetersPerSecond;
      System.out.println("Module " + i + " Drive Motor Voltage: " + voltage + " Drive Motor Velocity: " + velocity +
        " Drive Motor Output: " + driveSubsystem.getMaxSwerveModuleOutput()[i]);
      System.out.println("Module " + i + " Drive Motor kV: " + voltage / velocity + " volts / meters per sec");
      modulekV[i] = voltage / velocity;
      
    }

    System.out.println("Robot X Velocity: " + driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond);

    driveSubsystem.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getPose().getX()) > 3.5;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param voltage x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  public void setVoltage(double voltage) {
    this.voltage = MathUtil.clamp(voltage, -12, 12);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("X Voltage", () -> voltage, this::setVoltage);
    builder.addDoubleArrayProperty("Module kP", () -> modulekV, null);
  }
}
