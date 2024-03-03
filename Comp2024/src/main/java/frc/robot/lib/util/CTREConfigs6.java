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
import frc.robot.lib.math.Conversions;

public final class CTREConfigs6
{

  // Swerve modules

  public static TalonFXConfiguration exampleDriveFXConfig( )
  {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration( );

    // driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;
    // driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;

    // driveConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.driveSupplyCurrentLimit;
    // driveConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.driveSupplyCurrentThreshold;
    // driveConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.driveSupplyTimeThreshold;
    // driveConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.driveSupplyCurrentLimitEnable;

    // // driveConfig.CurrentLimits.*
    // // driveConfig.Feedback.*
    // // driveConfig.HardwareLimitSwitch.*
    // // driveConfig.MotionMagic.*

    // // driveConfig.MotorOutput.DutyCycleNeutralDeadband
    // driveConfig.MotorOutput.Inverted = SWConsts.driveMotorInvert;
    // driveConfig.MotorOutput.NeutralMode = SWConsts.driveNeutralMode;

    // driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;
    // driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;

    // driveConfig.Slot0.kS = SWConsts.driveFFKS;
    // driveConfig.Slot0.kV = SWConsts.driveFFKV;
    // driveConfig.Slot0.kP = SWConsts.driveKP;
    // driveConfig.Slot0.kI = SWConsts.driveKI;
    // driveConfig.Slot0.kD = SWConsts.driveKD;

    // driveConfig.SoftwareLimitSwitch.*

    return driveConfig;
  }

  public static TalonFXConfiguration exampleSteerFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );

    // angleConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.steerClosedLoopRamp;
    // angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.steerClosedLoopRamp;

    // angleConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.steerSupplyCurrentLimit;
    // angleConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.steerSupplyCurrentThreshold;
    // angleConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.steerSupplyTimeThreshold;
    // angleConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.steerSupplyCurrentLimitEnable;

    //  angleConfig.CurrentLimits.*
    //  angleConfig.Feedback.*
    //  angleConfig.HardwareLimitSwitch.*
    //  angleConfig.MotionMagic.*

    //  angleConfig.MotorOutput.DutyCycleNeutralDeadband
    // angleConfig.MotorOutput.Inverted = SWConsts.steerMotorInvert;
    // angleConfig.MotorOutput.NeutralMode = SWConsts.steerNeutralMode;

    // angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;
    // angleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;

    // angleConfig.Slot0.kS = SWConsts.steerFFKS;
    // angleConfig.Slot0.kV = SWConsts.steerFFKV;
    // angleConfig.Slot0.kP = SWConsts.steerKP;
    // angleConfig.Slot0.kI = SWConsts.steerKI;
    // angleConfig.Slot0.kD = SWConsts.steerKD;

    //  angleConfig.SoftwareLimitSwitch.*

    return angleConfig;
  }

  public static CANcoderConfiguration exampleCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    // config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // config.MagnetSensor.SensorDirection = SWConsts.steerCanCoderInvert;
    // if (Robot.isReal( ))
    //   config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? 0.0 : 0.0;
    // else
    //   config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default

    return config;
  }

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
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? 0.0 : -0.703613; // Rotations TODO: get comp value
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return config;
  }

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
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

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
    shooterConfig.Slot0.kV = 0.1129;                                // Voltage or duty cycle per requested RPS (velocity modes)
    shooterConfig.Slot0.kP = 0.5;                                   // Voltage or duty cycle per velocity error (velocity modes)
    shooterConfig.Slot0.kI = 0.0;                                   // Voltage or duty cycle per accumulated error
    shooterConfig.Slot0.kD = 0.0;                                   // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  // Climber

  public static TalonFXConfiguration climberFXConfig( )
  {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    climberConfig.CurrentLimits.SupplyCurrentLimit = 30.0;        // Amps
    climberConfig.CurrentLimits.SupplyCurrentThreshold = 30.0;    // Amps
    climberConfig.CurrentLimits.SupplyTimeThreshold = 0.001;      // Seconds
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberConfig.CurrentLimits.StatorCurrentLimit = 80.0;        // Amps
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
    climberConfig.Slot0.kP = 0.0;                                 // Voltage or duty cycle per velocity error (velocity modes)
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
