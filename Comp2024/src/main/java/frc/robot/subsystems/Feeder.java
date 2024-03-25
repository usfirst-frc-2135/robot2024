//
// Feeder Subystem - takes in Notes and feeds them into the Amp and Trap
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RotaryMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs5;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil5;
import frc.robot.lib.util.PhoenixUtil6;

//
// Feeder subsystem class
//
public class Feeder extends SubsystemBase
{

  // Constants
  private static final boolean kFeederMotorInvert  = false;     // Motor direction for positive input
  private static final double  kRotaryGearRatio    = 27.0;

  private static final double  kRotaryLengthMeters = 16.0;
  private static final double  kRotaryWeightKg     = 5.5;
  private static final double  kRotaryManualVolts  = 3.5;      // Motor voltage during manual operation (joystick)

  // Rotary constants
  private static final double  kToleranceDegrees   = 4.0;      // PID tolerance in degrees
  private static final double  kMMSafetyTimeout    = 2.0;

  // Member objects
  private final WPI_TalonSRX   m_feederRoller      = new WPI_TalonSRX(Ports.kCANID_FeederRoller);
  private final TalonFX        m_feederRotary      = new TalonFX(Ports.kCANID_FeederRotary);
  private final CANcoder       m_CANCoder          = new CANcoder(Ports.kCANID_FeederCANCoder);
  private final DigitalInput   m_noteInFeeder      = new DigitalInput(Ports.kDIO1_NoteInFeeder);

  // Declare module variables

  // Roller variables
  private boolean              m_fdRollerValid;      // Health indicator for motor 

  private static final double  kRollerSpeedAcquire = 0.5;
  private static final double  kRollerSpeedExpel   = -0.4;
  private static final double  kRollerSpeedHandoff = -0.4;

  // Rotary variables
  private boolean              m_fdRotaryValid;      // Health indicator for motor 
  private boolean              m_fdCCValid;          // Health indicator for CANCoder 
  private boolean              m_debug             = true;
  private double               m_currentDegrees    = 0.0; // Current angle in degrees
  private double               m_targetDegrees     = 0.0; // Target angle in degrees

  // Manual mode config parameters
  private VoltageOut           m_requestVolts      = new VoltageOut(0);
  private RotaryMode           m_rotaryMode        = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage   m_requestMMVolts    = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer            m_withinTolerance   = new Debouncer(0.060, DebounceType.kRising);
  private Debouncer            m_noteDetected      = new Debouncer(0.030, DebounceType.kBoth);
  private Timer                m_safetyTimer       = new Timer( ); // Safety timer for movements
  private boolean              m_moveIsFinished;     // Movement has completed (within tolerance)

  // Status signals
  private StatusSignal<Double> m_rotaryPosition    = m_feederRotary.getRotorPosition( );    // Not used in MM - uses CANcoder remote sensor
  private StatusSignal<Double> m_rotaryVelocity    = m_feederRotary.getRotorVelocity( );
  private StatusSignal<Double> m_rotaryCLoopError  = m_feederRotary.getClosedLoopError( );
  private StatusSignal<Double> m_rotarySupplyCur   = m_feederRotary.getSupplyCurrent( );
  private StatusSignal<Double> m_rotaryStatorCur   = m_feederRotary.getStatorCurrent( );
  private StatusSignal<Double> m_ccPosition        = m_CANCoder.getAbsolutePosition( );

  // Constructor

  public Feeder( )
  {
    setName("Feeder");
    setSubsystem("Feeder");

    // Roller motor init
    m_fdRollerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_feederRoller, "Feeder Roller", CTREConfigs5.intakeRollerConfig( ));
    m_feederRoller.setInverted(kFeederMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_feederRoller, "setInverted");

    // Rotary motor and CANcoder init
    m_fdRotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_feederRotary, "Feeder Rotary", CTREConfigs6.feederRotaryFXConfig( ));
    m_fdCCValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "Feeder Rotary", CTREConfigs6.feederRotaryCancoderConfig( ));

    Double ccRotations = getCANCoderRotations( );
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_fdRotaryValid)
      m_feederRotary.setPosition(Conversions.rotationsToInputRotations(ccRotations, kRotaryGearRatio)); // Not really used - CANcoder is remote sensor with absolute position

    m_rotaryPosition.setUpdateFrequency(50);
    if (m_debug)
    {
      m_rotaryVelocity.setUpdateFrequency(10);
      m_rotaryCLoopError.setUpdateFrequency(10);
      m_rotarySupplyCur.setUpdateFrequency(10);
      m_rotaryStatorCur.setUpdateFrequency(10);
    }
    m_ccPosition.setUpdateFrequency(10);

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    double rollerCurrent = (m_fdRollerValid) ? m_feederRoller.getStatorCurrent( ) : 0.0;
    SmartDashboard.putNumber("FD_rollerCur", rollerCurrent);

    // CANcoder is the primary (remote) sensor for Motion Magic
    m_currentDegrees = Conversions.rotationsToOutputDegrees(getRotaryRotations( ), kRotaryGearRatio);
    SmartDashboard.putNumber("FD_ccDegrees", Units.rotationsToDegrees(getCANCoderRotations( )));
    SmartDashboard.putNumber("FD_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("FD_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("FD_rotaryDegrees", m_currentDegrees); // reference is rotary encoder
    SmartDashboard.putBoolean("FD_noteInFeeder", m_noteInFeeder.get( ));
    if (m_debug && m_fdRotaryValid)
    {
      SmartDashboard.putNumber("IN_rotaryRps", m_rotaryVelocity.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_curError", m_rotaryCLoopError.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_rotSupCur", m_rotarySupplyCur.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_rotStatCur", m_rotaryStatorCur.refresh( ).getValue( ));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Initialize dashboard widgets

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_INValidRoller", m_fdRollerValid);
    SmartDashboard.putBoolean("HL_INValidNRotary", m_fdRotaryValid);
    SmartDashboard.putBoolean("HL_INValidCANCoder", m_fdCCValid);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    setRollerSpeed(FDRollerMode.STOP);
    setRotaryStopped( );

    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  private double getRotaryRotations( )
  {
    return (m_fdRotaryValid) ? m_rotaryPosition.refresh( ).getValue( ) : 0.0;
  }

  private double getCANCoderRotations( )
  {
    double ccRotations = (m_fdCCValid) ? m_ccPosition.refresh( ).getValue( ) : 0.0;
    ccRotations -= (Robot.isReal( )) ? 0.0 : 0.0; // 0.359130859;
    return ccRotations;
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public void setRollerSpeed(FDRollerMode mode)
  {
    double output = 0.0;

    if (mode == FDRollerMode.HOLD)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_feederRoller.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          output = 0.0;
          break;
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
        case HANDOFF :
          output = kRollerSpeedHandoff;
          break;
      }
      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_feederRoller.set(output);
    }
  }

  public double getFeederPosition( )
  {
    return m_currentDegrees;
  }

  public boolean isNoteDetected( )
  {
    return m_noteDetected.calculate(m_noteInFeeder.get( ));
  }

  public void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_feederRotary.setControl(m_requestVolts.withOutput(0.0));
  }

  private boolean isMoveValid(double degrees)
  {
    return (degrees >= INConsts.kRotaryAngleMin) && (degrees <= INConsts.kRotaryAngleMax);
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveRotaryWithJoystick(double axisValue)
  {
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > INConsts.kRotaryAngleMin))
      newMode = INConsts.RotaryMode.INBOARD;
    else if ((axisValue > 0.0) && (m_currentDegrees < INConsts.kRotaryAngleMax))
      newMode = INConsts.RotaryMode.OUTBOARD;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_rotaryMode)
    {
      m_rotaryMode = newMode;
      DataLogManager.log(String.format("%s: move %s %.1f deg %s", getSubsystem( ), m_rotaryMode, getFeederPosition( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_feederRotary.setControl(m_requestVolts.withOutput(axisValue * kRotaryManualVolts));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveToPositionInit(double newAngle, boolean holdPosition)
  {
    m_safetyTimer.restart( );

    if (holdPosition)
      newAngle = getFeederPosition( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio);
        m_feederRotary.setControl(m_requestMMVolts.withPosition(targetRotations));
        DataLogManager
            .log(String.format("%s: Position move: %.1f -> %.1f degrees (%.3f -> %.3f)", getSubsystem( ), m_currentDegrees,
                m_targetDegrees, Conversions.degreesToInputRotations(m_currentDegrees, kRotaryGearRatio), targetRotations));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f degrees is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetDegrees, INConsts.kRotaryAngleMin, INConsts.kRotaryAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_targetDegrees));
    }
  }

  public void moveToPositionExecute( )
  {
    m_feederRotary
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
