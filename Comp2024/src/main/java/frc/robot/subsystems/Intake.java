//
// Intake Subystem - takes in Notes and delivers them to the Shooter
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.Constants.INConsts.RotaryMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs5;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil5;
import frc.robot.lib.util.PhoenixUtil6;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{
  // Constants
  private static final boolean      kRollerMotorInvert    = true;     // Motor direction for positive input

  private static final double       kRollerSpeedAcquire   = 0.5;
  private static final double       kRollerSpeedExpel     = -0.25;
  private static final double       kRollerSpeedToShooter = -0.6;

  private static final double       kRotaryGearRatio      = 27.41;
  private static final double       kLigament2dOffset     = 0.0;      // Offset from mechanism root for ligament
  private static final double       kManualSpeedVolts     = 3.5;      // Motor voltage during manual operation (joystick)
  private static final double       kAngleMin             = -90.0;
  private static final double       kAngleRetracted       = -88.0;
  private static final double       kAngleHandoff         = 0.0;
  private static final double       kAngleDeployed        = 112.0;
  private static final double       kAngleMax             = 115.0;

  private static final double       kToleranceDegrees     = 2.0;      // PID tolerance in degrees
  private static final double       kMMSafetyTimeout      = 3.0;

  // Device and simulation objects
  private static final WPI_TalonSRX m_rollerMotor         = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private static final TalonFX      m_rotaryMotor         = new TalonFX(Ports.kCANID_IntakeRotary);
  private static final CANcoder     m_CANCoder            = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private static final DigitalInput m_noteInIntake        = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState     m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANCoderSim         = m_CANCoder.getSimState( );
  private final SingleJointedArmSim m_armSim              =
      new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio, 1.0, 0.3, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech          = new Mechanism2d(2, 2);
  private final MechanismRoot2d     m_mechRoot            = m_rotaryMech.getRoot("Rotary", 1.0, 1.0);
  private final MechanismLigament2d m_mechLigament        =
      m_mechRoot.append(new MechanismLigament2d("intake", 0.5, kLigament2dOffset, 6, new Color8Bit(Color.kPurple)));

  // Declare module variables
  private static boolean            m_isComp;

  // Roller variables
  private boolean                   m_rollerValid;     // Health indicator for motor 

  // Rotary variables
  private boolean                   m_rotaryValid;     // Health indicator for motor 
  private boolean                   m_ccValid;         // Health indicator for CANCoder 
  private boolean                   m_debug               = true;
  private static double             m_currentDegrees      = 0.0; // Current angle in degrees
  private double                    m_targetDegrees       = 0.0; // Target angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts        = new VoltageOut(0).withEnableFOC(false);
  private RotaryMode                m_rotaryMode          = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_requestMMVolts      = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                 m_withinTolerance     = new Debouncer(0.060, DebounceType.kRising);
  private boolean                   m_moveIsFinished;  // Movement has completed (within tolerance)
  private Timer                     m_safetyTimer         = new Timer( ); // Safety timer for movements

  // Status signals
  private StatusSignal<Double>      m_rotaryPosition      = m_rotaryMotor.getRotorPosition( );
  private StatusSignal<Double>      m_rotaryVelocity      = m_rotaryMotor.getRotorVelocity( );
  private StatusSignal<Double>      m_rotaryCLoopError    = m_rotaryMotor.getClosedLoopError( );
  private StatusSignal<Double>      m_rotarySupplyCur     = m_rotaryMotor.getSupplyCurrent( );
  private StatusSignal<Double>      m_rotaryStatorCur     = m_rotaryMotor.getStatorCurrent( );
  private StatusSignal<Double>      m_ccPosition          = m_CANCoder.getAbsolutePosition( );

  // Constructor

  public Intake(boolean isComp)
  {
    setName("Intake");
    setSubsystem("Intake");
    m_isComp = isComp;

    m_rollerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Intake Roller", CTREConfigs5.intakeRollerConfig( ));
    SmartDashboard.putBoolean("HL_validIN", m_rollerValid);
    m_rollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");

    m_rotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Intake Rotary", CTREConfigs6.intakeRotaryFXConfig( ));
    m_ccValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "Intake Rotary", CTREConfigs6.intakeRotaryCancoderConfig( ));

    if (Robot.isReal( ))
      m_currentDegrees = Units.rotationsToDegrees(m_ccPosition.refresh( ).getValue( ));
    m_rotaryMotor.setPosition(Conversions.degreesToInputRotations(m_currentDegrees, kRotaryGearRatio));
    DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

    m_rotarySim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANCoderSim.Orientation = ChassisReference.Clockwise_Positive;

    m_rotaryPosition.setUpdateFrequency(50);
    if (m_debug)
    {
      m_rotaryVelocity.setUpdateFrequency(50);
      m_rotaryCLoopError.setUpdateFrequency(50);
      m_rotarySupplyCur.setUpdateFrequency(50);
      m_rotaryStatorCur.setUpdateFrequency(50);
    }
    m_ccPosition.setUpdateFrequency(50);

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    double currentDraw = m_rollerMotor.getStatorCurrent( );
    SmartDashboard.putNumber("INRoller_currentDraw", currentDraw);

    m_currentDegrees = getTalonFXDegrees( );
    SmartDashboard.putNumber("IN_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("IN_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("IN_CCpostion", Units.rotationsToDegrees(m_ccPosition.refresh( ).getValueAsDouble( )));
    SmartDashboard.putBoolean("IN_noteInIntake", m_noteInIntake.get( ));
    if (m_debug)
    {
      SmartDashboard.putNumber("IN_velocity", m_rotaryVelocity.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_curError", m_rotaryCLoopError.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_supplyCur", m_rotarySupplyCur.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_statorCur", m_rotaryStatorCur.refresh( ).getValue( ));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_rotarySim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANCoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInput(m_rotarySim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rotarySim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kRotaryGearRatio));
    m_rotarySim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kRotaryGearRatio));

    m_CANCoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_CANCoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    m_mechLigament.setAngle(kLigament2dOffset - m_currentDegrees);
  }

  // Initialize dashboard widgets

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_validRotary", m_rotaryValid);
    SmartDashboard.putBoolean("HL_validRotaryCC", m_ccValid);
    SmartDashboard.putData("RotaryMech", m_rotaryMech);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    setRollerSpeed(RollerMode.STOP);
    setRotaryStopped( );

    //m_currentDegrees = getTalonFXDegrees( );
    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  private double getTalonFXDegrees( )
  {
    return Conversions.rotationsToOutputDegrees(m_rotaryPosition.refresh( ).getValue( ), kRotaryGearRatio);
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public void setRollerSpeed(RollerMode mode)
  {
    String strName;
    double output = 0.0;

    switch (mode)
    {
      default :
      case STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case ACQUIRE :
        strName = "ACQUIRE";
        output = kRollerSpeedAcquire;
        break;
      case EXPEL :
        strName = "EXPEL";
        output = kRollerSpeedExpel;
        break;
    }
    DataLogManager.log(String.format("%s: Mode is now - %s", getSubsystem( ), strName));
    m_rollerMotor.set(output);
  }

  public double getRotaryPosition( )
  {
    return m_currentDegrees;
  }

  public boolean isNoteDetected( )
  {
    return m_noteInIntake.get( );
  }

  private boolean isWithinTolerance(double targetDegrees)
  {
    return (Math.abs(targetDegrees - m_currentDegrees) < kToleranceDegrees);
  }

  public void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  private boolean isMoveValid(double degrees)
  {
    return (degrees > kAngleMin) && (degrees < kAngleMax);
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveRotaryWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if (axisValue < 0.0)
    {
      if (m_currentDegrees > kAngleMin)
      {
        newMode = INConsts.RotaryMode.UP;
      }
      else
        rangeLimited = true;
    }
    else if (axisValue > 0.0)
    {
      if (m_currentDegrees < kAngleMax)
      {
        newMode = INConsts.RotaryMode.DOWN;
      }
      else
        rangeLimited = true;
    }

    if (rangeLimited)
      axisValue = 0.0;

    if (newMode != m_rotaryMode)
    {
      m_rotaryMode = newMode;
      DataLogManager.log(String.format("%s: move %s %.1f deg %s", getSubsystem( ), m_rotaryMode, getRotaryPosition( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_rotaryMotor.setControl(m_requestVolts.withOutput(axisValue * kManualSpeedVolts));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveToPositionInit(double newAngle, boolean holdPosition)
  {
    m_safetyTimer.restart( );

    if (holdPosition)
      newAngle = getRotaryPosition( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !isWithinTolerance(newAngle))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        m_rotaryMotor
            .setControl(m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio)));
        DataLogManager.log(String.format("%s: Position move: %.1f -> %.1f degrees (%.1f -> %.1f)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Conversions.degreesToInputRotations(m_currentDegrees, kRotaryGearRatio),
            Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio)));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetDegrees, kAngleMin, kAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_targetDegrees));
    }
  }

  public void moveToPositionExecute( )
  {
    m_rotaryMotor
        .setControl(m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio)));
  }

  public boolean moveToPositionIsFinished( )
  {
    boolean timedOut = m_safetyTimer.hasElapsed(kMMSafetyTimeout);
    double error = m_targetDegrees - m_currentDegrees;

    if (m_withinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_moveIsFinished)
        DataLogManager.log(String.format("%s: Position move finished - Current degrees: %.1f (error %.1f) - Time: %.3f %s",
            getSubsystem( ), m_currentDegrees, error, m_safetyTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_moveIsFinished = true;
    }

    return m_moveIsFinished;
  }

  public void moveToPositionEnd( )
  {
    m_safetyTimer.stop( );
  }

}
