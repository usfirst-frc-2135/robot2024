package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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
    TalonFXConfiguration intakeRotaryConfig = new TalonFXConfiguration( );
    return intakeRotaryConfig;
  }

  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    return config;
  }

}
