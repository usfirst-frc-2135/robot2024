// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExampleSmartMotorController implements MotorController
{
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  private static final double kEncoderCPR = 4096;

  // private final static double ks          = 0.0;   // Volts to overcome static friction
  // private final static double kf          = 0.0;   // Volts per velocity unit
  private final static double kp          = 0.8;      // (native units) 10% * 102.3 / 1023
  private final static double ki          = 0.0;
  private final static double kd          = 0.0;

  private int                 m_port;
  private WPI_TalonSRX        m_motor;
  private ElevSim             m_elevSim;
  private double              m_rotations;
  private double              m_rotPerSec;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  //  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int port)
  {
    m_port = port;

    // Create the Talon SRX object and the attached CTRE Mag encoder
    m_motor = new WPI_TalonSRX(port);
    m_motor.configFactoryDefault( );
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.selectProfileSlot(0, 0);

    m_motor.config_kP(0, kp);
    m_motor.config_kI(0, ki);
    m_motor.config_kD(0, kd);

    // Connect simulation object to the Talon SRX
    m_elevSim = new ElevSim(m_motor, kEncoderCPR);
  }

  /**
   * Periodic processing for this motor/controller
   * 
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_rotations = countsToRotations(m_motor.getSelectedSensorPosition(0));
    m_rotPerSec = countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);

    m_elevSim.periodic( );

    SmartDashboard.putNumber("SRX" + m_port + "-Rotations", m_rotations);                                 // Output shaft distance (rotations)
    SmartDashboard.putNumber("SRX" + m_port + "-Velocity", m_rotPerSec);                                  // Output shaft velocity (rps)
    SmartDashboard.putNumber("SRX" + m_port + "-normError", m_motor.getClosedLoopError( ) / kEncoderCPR); // Normalized error in shaft rotations 
    if (m_motor.getControlMode( ) == ControlMode.Position || m_motor.getControlMode( ) == ControlMode.Velocity)
      SmartDashboard.putNumber("SRX" + m_port + "-target", m_motor.getClosedLoopTarget( ));              // Normalized error in shaft rotations 
  }

  /**
   * Converts shaft rotations to encoder counts
   *
   * @param rotation
   *          The rotation value to be converted.
   * @return encoderCounts
   */
  private double rotationsToCounts(double rotation)
  {
    return rotation * kEncoderCPR;
  }

  /**
   * Converts encoder counts to shaft rotations
   *
   * @param encoderCounts
   *          The count value to be converted.
   * @return rotations
   */
  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / kEncoderCPR;
  }

  /**
   * Set the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *          The mode of the PID controller.
   * @param setpoint
   *          The controller setpoint.
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward)
  {
    ControlMode controlMode;

    switch (mode)
    {
      default :
      case kPosition :  // Position PID
        controlMode = ControlMode.Position;
        break;

      case kVelocity :  // Velocity PID
        controlMode = ControlMode.Velocity;
        setpoint /= 10; // Adjust for CTRE units
        break;

      case kMovementWitchcraft :  // Motion Magic profile TODO: not yet implemented
        controlMode = ControlMode.MotionMagic;
        break;
    }

    // Talon SRX is before gearbox, but CTRE mag encoder is after it (multiply by gear ratio)
    m_motor.set(controlMode, rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *          The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader)
  {}

  /**
   * Returns the encoder position in rotations.
   *
   * @return The current encoder position in rotations.
   */
  public double getEncoderPosition( )
  {
    return m_rotations;
  }

  /**
   * Returns the encoder velocity.
   *
   * @return The current encoder velocity in rotations per second.
   */
  public double getEncoderVelocity( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /**
   * Resets the encoder to zero distance.
   * 
   */
  public void resetEncoder( )
  {
    m_motor.setSelectedSensorPosition(0, 0, 0);
    m_elevSim.reset( );
  }

  /**
   * Set motor to a known percent output value.
   *
   * @param percentOutput
   *          Percent output in range [-1.0, 1.0].
   */
  @Override
  public void set(double percentOutput)
  {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Returns the motor setting in use.
   *
   * @return percentOutput in range [-1.0, 1.0].
   */
  @Override
  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /**
   * Set motor inverted state.
   *
   * @param isInverted
   *          Invert motor if true
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_motor.setInverted(isInverted);
  }

  /**
   * Get motor inverted state.
   *
   * @return invertedState
   */
  @Override
  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  /**
   * Set motor to the disabled state.
   * 
   */
  @Override
  public void disable( )
  {
    m_motor.set(ControlMode.Disabled, 0);
  }

  /**
   * Set motor to the stopped state.
   * 
   */
  @Override
  public void stopMotor( )
  {
    set(0.0);
  }

}
