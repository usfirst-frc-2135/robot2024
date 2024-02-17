package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.Ports;
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
    // inRotaryConfig.ClosedLoopRamps.*

    // Current limit settings
    inRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    inRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 25.0;
    inRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;
    inRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRotaryConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    inRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // inRotaryConfig.Feedback.FeedbackRemoteSensorID = Ports.kCANID_IntakeCANCoder;
    // inRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // inRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    // inRotaryConfig.Feedback.RotorToSensorRatio = 27.41;

    // Hardware limit switches
    // inRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 22.0;
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 44.0;
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
    inRotaryConfig.Slot0.kV = 0.1129 / 4;
    inRotaryConfig.Slot0.kP = 0.0;
    inRotaryConfig.Slot0.kI = 0.0;
    inRotaryConfig.Slot0.kD = 0.0;

    // Software limit switches
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations((robotType) ? kAngleMin : kAngleMin);
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations((robotType) ? kAngleMin : kAngleMin);
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? 0.0 : -0.891113; // TODO: get comp value
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default

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
