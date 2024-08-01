// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TuneVelocityPCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private double velocity;
  private double velocityP;
  private double finalVelocity;

  private SwerveModuleState[] state;

  /** Creates a new TuneVelocityPCommand. */
  public TuneVelocityPCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    velocity = 0;
    velocityP = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    Shuffleboard.getTab("5: Rotation P").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Update the Motor P to the values set in the ShuffleBOard tab
    driveSubsystem.setDriveP(velocityP);

    // Set wheels to face forward.
    driveSubsystem.setVelocity(0);
    
    // Reset odometry
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(new Pose2d());
    
    // Set speed based on the velocity set in Shuffleboard
    state = new SwerveModuleState[] {
      new SwerveModuleState(velocity, new Rotation2d()),
      new SwerveModuleState(velocity, new Rotation2d()),
      new SwerveModuleState(velocity, new Rotation2d()),
      new SwerveModuleState(velocity, new Rotation2d())
    };

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setModuleStates(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Get the final velocity before stop the motors.
    finalVelocity = driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;

    // Stop the robot.
    driveSubsystem.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getPose().getX()) > 3.5;
  }

  /**
   * The p value to set for the velocity PID config.
   * 
   * @param velocityP in units of voltage / rpm
   */
  private void setVelocityP(double velocityP) {
    this.velocityP = velocityP;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  private void setVelocity(double velocity) {
    this.velocity = MathUtil.clamp(velocity, -1 * DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxSpeedMetersPerSecond);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("Velocity P", () -> velocityP, this::setVelocityP);
    builder.addDoubleProperty("Final Velocity", () -> finalVelocity, null);
    builder.addDoubleProperty("Current Velocity", () -> driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond, null);
  }

}
