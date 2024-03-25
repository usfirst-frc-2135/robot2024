
// Phoenix 6 configurations

package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.INConsts;
import frc.robot.Robot;

public final class CTREConfigs6
{

  // Swerve module configs built into subsystem

  // Intake

  public static TalonFXConfiguration intakeRotaryFXConfig( )
  {
    TalonFXConfiguration inRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    inRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 25.0;   // Amps
    inRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;     // Seconds
    inRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRotaryConfig.CurrentLimits.StatorCurrentLimit = 100.0;       // Amps
    inRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // inRotaryConfig.Feedback.FeedbackRemoteSensorID = Ports.kCANID_IntakeCANCoder;
    // inRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // inRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    // inRotaryConfig.Feedback.RotorToSensorRatio = 27.41;

    // Hardware limit switches
    // inRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0;  // Rotations / second
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 90.0;    // Rotations / second ^ 2
    inRotaryConfig.MotionMagic.MotionMagicJerk = 360.0;           // Rotations / second ^ 3

    // Motor output settings
    inRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;  // Percentage
    inRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    inRotaryConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0; // Seconds to ramp
    // inRotaryConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod      // Seconds to ramp
    inRotaryConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;   // Seconds to ramp

    // Slot settings
    inRotaryConfig.Slot0.kS = 0.0;                                // Voltage or duty cylce to overcome static friction
    inRotaryConfig.Slot0.kV = 0.1129;                             // Voltage or duty cycle per requested RPS (velocity modes)
    inRotaryConfig.Slot0.kP = 0.2;                                // Voltage or duty cycle per velocity error (velocity modes)
    inRotaryConfig.Slot0.kI = 0.0;                                // Voltage or duty cycle per accumulated error
    inRotaryConfig.Slot0.kD = 0.0;                                // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(INConsts.kRotaryAngleMin);  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(INConsts.kRotaryAngleMax);  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? 0.511719 : -0.703613;
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return config;
  }

  // Shooter

  public static TalonFXConfiguration shooterFXConfig( )
  {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 35.0;        // Amps
    shooterConfig.CurrentLimits.SupplyCurrentThreshold = 35.0;    // Amps
    shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.001;      // Seconds
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.CurrentLimits.StatorCurrentLimit = 100.0;        // Amps
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // Motor output settings
    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;  // Seconds to ramp
    // inRotaryConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod        // Seconds to ramp
    shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;    // Seconds to ramp

    // Slot settings
    shooterConfig.Slot0.kS = 0.0;                                   // Voltage or duty cylce to overcome static friction
    shooterConfig.Slot0.kV = 0.1140;                                // Voltage or duty cycle per requested RPS (velocity modes)
    shooterConfig.Slot0.kP = 0.25;                                   // Voltage or duty cycle per velocity error (velocity modes)
    shooterConfig.Slot0.kI = 0.0;                                   // Voltage or duty cycle per accumulated error
    shooterConfig.Slot0.kD = 0.0;                                   // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  // Feeder

  public static TalonFXConfiguration feederRotaryFXConfig( )
  {
    TalonFXConfiguration fdRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // fdRotaryConfig.ClosedLoopGeneral.*
    // fdRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    fdRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    fdRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 25.0;   // Amps
    fdRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;     // Seconds
    fdRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    fdRotaryConfig.CurrentLimits.StatorCurrentLimit = 100.0;       // Amps
    fdRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // fdRotaryConfig.Feedback.FeedbackRemoteSensorID = Ports.kCANID_IntakeCANCoder;
    // fdRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // fdRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    // fdRotaryConfig.Feedback.RotorToSensorRatio = 27.41;

    // Hardware limit switches
    // fdRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    fdRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0;  // Rotations / second
    fdRotaryConfig.MotionMagic.MotionMagicAcceleration = 90.0;    // Rotations / second ^ 2
    fdRotaryConfig.MotionMagic.MotionMagicJerk = 360.0;           // Rotations / second ^ 3

    // Motor output settings
    fdRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;  // Percentage
    fdRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fdRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    fdRotaryConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0; // Seconds to ramp
    // fdRotaryConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod      // Seconds to ramp
    fdRotaryConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;   // Seconds to ramp

    // Slot settings
    fdRotaryConfig.Slot0.kS = 0.0;                                // Voltage or duty cylce to overcome static friction
    fdRotaryConfig.Slot0.kV = 0.1129;                             // Voltage or duty cycle per requested RPS (velocity modes)
    fdRotaryConfig.Slot0.kP = 0.2;                                // Voltage or duty cycle per velocity error (velocity modes)
    fdRotaryConfig.Slot0.kI = 0.0;                                // Voltage or duty cycle per accumulated error
    fdRotaryConfig.Slot0.kD = 0.0;                                // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    fdRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(INConsts.kRotaryAngleMin);  // Rotations
    // fdRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fdRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(INConsts.kRotaryAngleMax);  // Rotations
    // fdRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return fdRotaryConfig;
  }

  public static CANcoderConfiguration feederRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? 0.511719 : -0.703613;
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return config;
  }

  // Climber

  public static TalonFXConfiguration climberFXConfig( )
  {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    climberConfig.CurrentLimits.SupplyCurrentLimit = 80.0;        // Amps
    climberConfig.CurrentLimits.SupplyCurrentThreshold = 80.0;    // Amps
    climberConfig.CurrentLimits.SupplyTimeThreshold = 0.001;      // Seconds
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberConfig.CurrentLimits.StatorCurrentLimit = 800.0;        // Amps
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // climberConfig.Feedback.*

    // Hardware limit switches
    // climberConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 79.75;  // Rotations / second
    climberConfig.MotionMagic.MotionMagicAcceleration = 708.9;    // Rotations / second ^ 2
    climberConfig.MotionMagic.MotionMagicJerk = 3544;             // Rotations / second ^ 3

    // Motor output settings
    climberConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;   // Percentage
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    climberConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;  // Seconds to ramp
    // exConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod            // Seconds to ramp
    climberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;    // Seconds to ramp

    // Slot settings
    climberConfig.Slot0.kS = 0.0;                                 // Voltage or duty cylce to overcome static friction
    climberConfig.Slot0.kV = 0.1129;                              // Voltage or duty cycle per requested RPS (velocity modes)
    climberConfig.Slot0.kP = 4.80;                                 // Voltage or duty cycle per velocity error (velocity modes)
    climberConfig.Slot0.kI = 0.0;                                 // Voltage or duty cycle per accumulated error
    climberConfig.Slot0.kD = 0.0;                                 // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.inchesToWinchRotations(0.0, 0.432);   // Rotations
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.inchesToWinchRotations(18.25, 0.432); // Rotations
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return climberConfig;
  }
}
