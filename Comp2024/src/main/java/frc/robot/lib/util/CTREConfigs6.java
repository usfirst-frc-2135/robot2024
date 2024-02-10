package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.SWConsts;
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

    // //  angleConfig.CurrentLimits.*
    // //  angleConfig.Feedback.*
    // //  angleConfig.HardwareLimitSwitch.*
    // //  angleConfig.MotionMagic.*

    // //  angleConfig.MotorOutput.DutyCycleNeutralDeadband
    // angleConfig.MotorOutput.Inverted = SWConsts.steerMotorInvert;
    // angleConfig.MotorOutput.NeutralMode = SWConsts.steerNeutralMode;

    // angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;
    // angleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;

    // angleConfig.Slot0.kS = SWConsts.steerFFKS;
    // angleConfig.Slot0.kV = SWConsts.steerFFKV;
    // angleConfig.Slot0.kP = SWConsts.steerKP;
    // angleConfig.Slot0.kI = SWConsts.steerKI;
    // angleConfig.Slot0.kD = SWConsts.steerKD;

    // //  angleConfig.SoftwareLimitSwitch.*

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

    // // Motion Magic config parameters
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
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    inRotaryConfig.MotionMagic.MotionMagicJerk = 0.0;

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
    inRotaryConfig.Slot0.kV = 0.0;
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

}
