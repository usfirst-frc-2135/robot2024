package frc.robot.lib.util;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
    // // config.MagnetSensor.MagnetOffset 

    return config;
  }

  public static TalonFXConfiguration intakeRotaryFXConfig( )
  {
    TalonFXConfiguration inRotaryConfig = new TalonFXConfiguration( );

    // Motion Magic config parameters
    // public static final double kMMVelocity = 79.75;          // 10/7/23 Tuned! Wrist motion magic velocity (75% of max motor RPM)
    // public static final double kMMAcceleration = 472.6;          // 10/7/23 Tuned! Wrist motion magic acceleration (target velocity in 1/2s)
    // public static final double kMMSCurveStrength = kMMAcceleration * 4.0; // Elbow motion magic jerk limit (1/4 of acceleration time)
    // public static final double kS = 0.0;            // Voltage constant to overcome friction
    // public static final double kV = 0.1129;         // Voltage constant per desired RPM
    // public static final double kPidKp = 1.350;          // Wrist PID proportional constant
    // public static final double kPidKi = 0.0;            // Wrist PID integral constant
    // public static final double kPidKd = 0.0;            // Wrist PID derivative constant

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*

    // Current limit settings
    inRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    inRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 25.0;
    inRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;
    inRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRotaryConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    inRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // inRotaryConfig.Feedback.*

    // Hardware limit switches
    // inRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 26.58;
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 472.6;
    inRotaryConfig.MotionMagic.MotionMagicJerk = 2363;

    // Motor output settings
    inRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    inRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    inRotaryConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // inRotaryConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    inRotaryConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    inRotaryConfig.Slot0.kS = 0.0;
    inRotaryConfig.Slot0.kV = 0.1129 / 4;
    inRotaryConfig.Slot0.kP = 0.0;
    inRotaryConfig.Slot0.kI = 0.0;
    inRotaryConfig.Slot0.kD = 0.0;

    // Software limit switches
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     Conversions.degreesToInputRotations(WRConsts.kAngleMin, WRConsts.kGearRatio);
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     Conversions.degreesToInputRotations(WRConsts.kAngleMax, WRConsts.kGearRatio);
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.MagnetOffset = -0.891113;

    return config;
  }

  public static TalonFXConfiguration shooterFXConfig( )
  {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration( );

    // public static final int kVelocityMeasWindow = 1;
    // public static final SensorVelocityMeasPeriod kVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_10Ms;
    // public static final double kFlywheelPidKf = 0.04775;
    // public static final double kFlywheelPidKp = 0.2;
    // public static final double kFlywheelPidKi = 0.0;
    // public static final double kFlywheelPidKd = 0.0;
    // public static final double kFlywheelNeutralDeadband = 0.01;

    // shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    // shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    shooterConfig.CurrentLimits.SupplyCurrentThreshold = 35.0;
    shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.001;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    // shooterConfig.MotorOutput.Inverted = false;
    // shooterConfig.MotorOutput.NeutralMode = NeutralMode.Coast;

    shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

    shooterConfig.Slot0.kS = 0.0;
    shooterConfig.Slot0.kV = 0.1129;
    shooterConfig.Slot0.kP = 0.0;
    shooterConfig.Slot0.kI = 0.0;
    shooterConfig.Slot0.kD = 0.0;

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  // Climber

  public static TalonFXConfiguration climberLengthFXConfig( )
  {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    climberConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    climberConfig.CurrentLimits.SupplyCurrentThreshold = 20.0;
    climberConfig.CurrentLimits.SupplyTimeThreshold = 0.001;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberConfig.CurrentLimits.StatorCurrentLimit = 45.0;
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // climberConfig.Feedback.*

    // Hardware limit switches
    // climberConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 79.75;
    climberConfig.MotionMagic.MotionMagicAcceleration = 708.9;
    climberConfig.MotionMagic.MotionMagicJerk = 3544;

    // Motor output settings
    climberConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Closed loop settings

    // Open Loop settings
    climberConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    // exConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod
    climberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    // Slot settings
    climberConfig.Slot0.kS = 0.0;
    climberConfig.Slot0.kV = 0.1129;
    climberConfig.Slot0.kP = 0.0451;
    climberConfig.Slot0.kI = 0.001;
    climberConfig.Slot0.kD = 0.4514;

    // Software limit switches
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.inchesToWinchRotations(0.0, 0.432);
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.inchesToWinchRotations(18.25, 0.432);
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return climberConfig;
  }
}
