package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConsts;
import frc.robot.Constants.IntakeRotorConsts;
import frc.robot.Constants.SWConsts;
import frc.robot.Constants.FeederRollerConsts;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;

public final class CTREConfigs6
{

  // Swerve modules

  public static TalonFXConfiguration swerveDriveFXConfig( )
  {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration( );

    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.driveClosedLoopRamp;

    driveConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.driveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.driveSupplyCurrentThreshold;
    driveConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.driveSupplyTimeThreshold;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.driveSupplyCurrentLimitEnable;

    // driveConfig.CurrentLimits.*
    // driveConfig.Feedback.*
    // driveConfig.HardwareLimitSwitch.*
    // driveConfig.MotionMagic.*

    // driveConfig.MotorOutput.DutyCycleNeutralDeadband
    driveConfig.MotorOutput.Inverted = SWConsts.driveMotorInvert;
    driveConfig.MotorOutput.NeutralMode = SWConsts.driveNeutralMode;

    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.driveOpenLoopRamp;

    driveConfig.Slot0.kS = SWConsts.driveFFKS;
    driveConfig.Slot0.kV = SWConsts.driveFFKV;
    driveConfig.Slot0.kP = SWConsts.driveKP;
    driveConfig.Slot0.kI = SWConsts.driveKI;
    driveConfig.Slot0.kD = SWConsts.driveKD;

    // driveConfig.SoftwareLimitSwitch.*

    return driveConfig;
  }

  public static TalonFXConfiguration swerveAngleFXConfig( )
  {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration( );

    angleConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SWConsts.steerClosedLoopRamp;
    angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SWConsts.steerClosedLoopRamp;

    angleConfig.CurrentLimits.SupplyCurrentLimit = SWConsts.steerSupplyCurrentLimit;
    angleConfig.CurrentLimits.SupplyCurrentThreshold = SWConsts.steerSupplyCurrentThreshold;
    angleConfig.CurrentLimits.SupplyTimeThreshold = SWConsts.steerSupplyTimeThreshold;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = SWConsts.steerSupplyCurrentLimitEnable;

    //  angleConfig.CurrentLimits.*
    //  angleConfig.Feedback.*
    //  angleConfig.HardwareLimitSwitch.*
    //  angleConfig.MotionMagic.*

    //  angleConfig.MotorOutput.DutyCycleNeutralDeadband
    angleConfig.MotorOutput.Inverted = SWConsts.steerMotorInvert;
    angleConfig.MotorOutput.NeutralMode = SWConsts.steerNeutralMode;

    angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;
    angleConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SWConsts.steerOpenLoopRamp;

    angleConfig.Slot0.kS = SWConsts.steerFFKS;
    angleConfig.Slot0.kV = SWConsts.steerFFKV;
    angleConfig.Slot0.kP = SWConsts.steerKP;
    angleConfig.Slot0.kI = SWConsts.steerKI;
    angleConfig.Slot0.kD = SWConsts.steerKD;

    //  angleConfig.SoftwareLimitSwitch.*

    return angleConfig;
  }

  public static CANcoderConfiguration swerveCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SWConsts.steerCanCoderInvert;
    // config.MagnetSensor.MagnetOffset 

    return config;
  }
}

// elConfig.ClosedLoopGeneral.*
// elConfig.ClosedLoopRamps.*

// Current limit settings

// Feedback settings
// elConfig.Feedback.*

// Hardware limit switches
// elConfig.HardwareLimitSwitch.*

// Motion Magic settings

// Motor output settings

// Closed loop settings

// Open Loop settings

// elConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod

// Slot settings

// Software limit switches

// Closed Loop settings
// wrConfig.ClosedLoopGeneral.*
// wrConfig.ClosedLoopRamps.*

// Current limit settings

// Feedback settings
// wrConfig.Feedback.*

// Hardware limit switches
// wrConfig.HardwareLimitSwitch.*

// Motion Magic settings

// Closed loop settings

// Open Loop settings

// wrConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod

// Slot settings

// Software limit switches

// Closed Loop settings
// exConfig.ClosedLoopGeneral.*
// exConfig.ClosedLoopRamps.*

// Current limit settings

// Feedback settings
// exConfig.Feedback.*

// Hardware limit switches
// exConfig.HardwareLimitSwitch.*

// Motion Magic settings

// Motor output settings

// Closed loop settings

// Open Loop settings

// exConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod

// Slot settings

// Software limit switches
