// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final CANSparkFlex m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private final SimpleMotorFeedforward drivMotorFeedforward;
  private final double drivekS = 0.2;
  private final double drivekV = 1.95;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    drivMotorFeedforward = new SimpleMotorFeedforward(drivekS, drivekV);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
      optimizedDesiredState.speedMetersPerSecond, 
      ControlType.kVelocity,
      0,
      drivMotorFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond),
      ArbFFUnits.kVoltage);

    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getDriveVelocity(){
    return m_drivingEncoder.getVelocity();
  }

  // Methods added for tuning / drive characterization \\

  /**
   * Method to drive straight at a set drive motor voltage.</p>
   * Steer motor set to zero degrees.  
   * @param volts -12 to 12 volts
   */
  public void setDriveStraightVoltage(double volts) {
    m_drivingSparkMax.setVoltage(volts);
    m_turningPIDController.setReference(m_chassisAngularOffset, ControlType.kPosition);
  }

  // TODO is this the correct voltage?
  /**
   * @return voltage of the motor
   */
  public double getDriveMotorVoltage(){
    return m_drivingSparkMax.getBusVoltage();
  }

  /**
   * @return duty cycle from -1 to 1
   */
  public double getDriveMotorOutput(){
    return m_drivingSparkMax.getAppliedOutput();
  }

  /**
   *  Set drive motor IdleMode
   * @param mode coast or brake
   * @return kOk if set, or kError if not set properly
   */
  public REVLibError setDriveMotorIdleMode(IdleMode mode){
    for (int i = 0; i < 5; i++) {
      if (m_drivingSparkMax.setIdleMode(mode) == REVLibError.kOk) {
        return REVLibError.kOk;
      };
    }
    return REVLibError.kError;
  }

  /**
   *  Set drive motor P
   * @param PValue P value for PID
   * @return kOk if set, or kError if not set properly
   */
  public REVLibError setDriveMotorP(double PValue){
    for (int i = 0; i < 5; i++) {
      if (m_drivingPIDController.setP(PValue) == REVLibError.kOk) {
        return REVLibError.kOk;
      };
    }
    return REVLibError.kError;
  }

  /**
   *  Set angle motor P
   * @param PValue P value for PID
   * @return kOk if set, or kError if not set properly
   */
  public REVLibError setAngleMotorP(double PValue){
    for (int i = 0; i < 5; i++) {
      if ( m_turningPIDController.setP(PValue) == REVLibError.kOk) {
        return REVLibError.kOk;
      };
    }
    return REVLibError.kError;
  }
}
