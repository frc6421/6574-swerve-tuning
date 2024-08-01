// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TuneRotationPCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private double rotation;
  private double rotationP;
  private double finalRotation0;
  private double finalRotation1;
  private double finalRotation2;
  private double finalRotation3;

  private SwerveModuleState[] state;

  /** Creates a new TuneVelocityPCommand. */
  public TuneRotationPCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    rotation = 0;
    rotationP = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    Shuffleboard.getTab("6: Rotation P").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Update the Motor P to the values set in the ShuffleBOard tab
    driveSubsystem.setAngleP(rotationP);

    // Set wheels to face forward.
    driveSubsystem.setVelocity(0);
    
    // Reset odometry
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(new Pose2d());
    
    // Set speed based on the velocity set in Shuffleboard
    state = new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(rotation))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(rotation))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(rotation))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(rotation)))
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
    finalRotation0 = driveSubsystem.getModulePositions()[0].angle.getDegrees();
    finalRotation1 = driveSubsystem.getModulePositions()[1].angle.getDegrees();
    finalRotation2 = driveSubsystem.getModulePositions()[2].angle.getDegrees();
    finalRotation3 = driveSubsystem.getModulePositions()[3].angle.getDegrees();

    // Stop the robot.
    driveSubsystem.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return false;
  }

  /**
   * The p value to set for the velocity PID config.
   * 
   * @param velocityP in units of voltage / rpm
   */
  private void setRotationP(double velocityP) {
    this.rotationP = velocityP;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param rotation x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  private void setRotation(double rotation) {
    this.rotation = rotation;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Rotation", () -> rotation, this::setRotation);
    builder.addDoubleProperty("Rotation P", () -> rotationP, this::setRotationP);
    builder.addDoubleProperty("Final Rotation 0", () -> finalRotation0, null);
    builder.addDoubleProperty("Final Rotation 1", () -> finalRotation1, null);
    builder.addDoubleProperty("Final Rotation 2", () -> finalRotation2, null);
    builder.addDoubleProperty("Final Rotation 3", () -> finalRotation3, null);
    builder.addDoubleProperty("Current Velocity", () -> driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond, null);
  }

}
