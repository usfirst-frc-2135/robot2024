//
// Feeder Subystem - takes in Notes and feeds them into the Amp and Trap
//
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FDConsts;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.FDConsts.RotaryMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Feeder subsystem class
 */
public class Feeder extends SubsystemBase
{

  // Constants
  private static final boolean      kFeederMotorInvert  = true;     // Motor direction for positive input

  private static final double       kRollerSpeedScore   = -0.5;
  private static final double       kRollerSpeedHandoff = -0.37;

  private static final double       kLigament2dOffset   = 90.0;      // Offset from mechanism root for ligament
  private static final double       kRotaryGearRatio    = 27.0;
  private static final double       kRotaryLengthMeters = 0.5;
  private static final double       kRotaryWeightKg     = 4.0;
  private static final double       kRotaryManualVolts  = 3.5;      // Motor voltage during manual operation (joystick)

  // Rotary constants
  private static final double       kToleranceDegrees   = 4.0;      // PID tolerance in degrees
  private static final double       kMMSafetyTimeout    = 2.0;      // Seconds allowed for a Motion Magic movement (TODO: TUNE ME)

  // Device and simulation objects
  private final WPI_TalonSRX        m_rollerMotor       = new WPI_TalonSRX(Ports.kCANID_FeederRoller);
  private final TalonFX             m_rotaryMotor       = new TalonFX(Ports.kCANID_FeederRotary);
  private final CANcoder            m_CANcoder          = new CANcoder(Ports.kCANID_FeederCANcoder);
  private final DigitalInput        m_noteInFeeder      = new DigitalInput(Ports.kDIO1_NoteInFeeder);

  private final TalonFXSimState     m_rotarySim         = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANcoderSim       = m_CANcoder.getSimState( );
  private final SingleJointedArmSim m_armSim            = new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech        = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d     m_mechRoot          = m_rotaryMech.getRoot("Rotary", 0.5, 0.5);
  private final MechanismLigament2d m_mechLigament      =
      m_mechRoot.append(new MechanismLigament2d("feeder", 0.5, kLigament2dOffset, 6, new Color8Bit(Color.kBlue)));

  // Roller variables
  private boolean                   m_fdRollerValid;      // Health indicator for motor 
  private Debouncer                 m_noteDebouncer     = new Debouncer(0.030, DebounceType.kBoth);
  private boolean                   m_noteDetected;       // Detection state of note in rollers

  // Rotary variables
  private boolean                   m_fdRotaryValid;      // Health indicator for motor 
  private boolean                   m_fdCCValid;          // Health indicator for CANcoder 
  private boolean                   m_debug             = true;
  private double                    m_currentDegrees    = 0.0; // Current angle in degrees
  private double                    m_targetDegrees     = 0.0; // Target angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts      = new VoltageOut(0);
  private RotaryMode                m_rotaryMode        = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_requestMMVolts    = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                 m_withinTolerance   = new Debouncer(0.060, DebounceType.kRising);
  private Timer                     m_safetyTimer       = new Timer( ); // Safety timer for movements
  private boolean                   m_moveIsFinished;     // Movement has completed (within tolerance)

  // Status signals
  private StatusSignal<Double>      m_rotaryPosition    = m_rotaryMotor.getRotorPosition( );    // Not used in MM - uses CANcoder remote sensor
  private StatusSignal<Double>      m_rotaryCLoopError  = m_rotaryMotor.getClosedLoopError( );
  private StatusSignal<Double>      m_rotarySupplyCur   = m_rotaryMotor.getSupplyCurrent( );
  private StatusSignal<Double>      m_rotaryStatorCur   = m_rotaryMotor.getStatorCurrent( );
  private StatusSignal<Double>      m_ccPosition        = m_CANcoder.getAbsolutePosition( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Feeder( )
  {
    setName("Feeder");
    setSubsystem("Feeder");

    // Roller motor init
    m_fdRollerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Feeder Roller", CTREConfigs5.feederRollerConfig( ));
    m_rollerMotor.setInverted(kFeederMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");

    // Rotary motor and CANcoder init
    m_fdRotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Feeder Rotary", CTREConfigs6.feederRotaryFXConfig( ));
    m_fdCCValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, "Feeder Rotary", CTREConfigs6.feederRotaryCancoderConfig( ));

    Double ccRotations = getCANcoderRotations( );
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_fdRotaryValid)
      m_rotaryMotor.setPosition(Conversions.rotationsToInputRotations(ccRotations, kRotaryGearRatio)); // Not really used - CANcoder is remote sensor with absolute position

    // Simulation object initialization
    m_rotarySim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANcoderSim.Orientation = ChassisReference.Clockwise_Positive;

    m_rotaryPosition.setUpdateFrequency(50);
    if (m_debug)
      BaseStatusSignal.setUpdateFrequencyForAll(20, m_rotaryCLoopError, m_rotarySupplyCur, m_rotaryStatorCur);
    m_ccPosition.setUpdateFrequency(10);

    initSmartDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    double rollerCurrent = (m_fdRollerValid) ? m_rollerMotor.getStatorCurrent( ) : 0.0;
    SmartDashboard.putNumber("FD_rollerCur", rollerCurrent);

    m_noteDetected = m_noteDebouncer.calculate(m_noteInFeeder.get( ));

    // CANcoder is the primary (remote) sensor for Motion Magic
    m_currentDegrees = Conversions.rotationsToOutputDegrees(getRotaryRotations( ), kRotaryGearRatio);
    SmartDashboard.putNumber("FD_ccDegrees", Units.rotationsToDegrees(getCANcoderRotations( )));
    SmartDashboard.putNumber("FD_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("FD_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("FD_rotaryDegrees", m_currentDegrees); // reference is rotary encoder
    SmartDashboard.putBoolean("FD_noteDetected", m_noteDetected);
    if (m_debug && m_fdRotaryValid)
    {
      BaseStatusSignal.refreshAll(m_rotaryCLoopError, m_rotarySupplyCur, m_rotaryStatorCur);
      SmartDashboard.putNumber("FD_curError", m_rotaryCLoopError.getValue( ));
      SmartDashboard.putNumber("FD_rotSupCur", m_rotarySupplyCur.getValue( ));
      SmartDashboard.putNumber("FD_rotStatCur", m_rotaryStatorCur.getValue( ));
    }
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_rotarySim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANcoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInputVoltage(m_rotarySim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rotarySim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kRotaryGearRatio));
    m_rotarySim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kRotaryGearRatio));

    m_CANcoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_CANcoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    m_mechLigament.setAngle(kLigament2dOffset - m_currentDegrees);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_FDValidRoller", m_fdRollerValid);
    SmartDashboard.putBoolean("HL_FDValidRotary", m_fdRotaryValid);
    SmartDashboard.putBoolean("HL_FDValidCANcoder", m_fdCCValid);
    SmartDashboard.putData("FDRotaryMech", m_rotaryMech);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    setRollerMode(FDRollerMode.STOP);
    setRotaryStopped( );

    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil5.getInstance( ).talonSRXPrintFaults(m_rollerMotor, "FeederRoller");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rotaryMotor, "FeederRotary");
    PhoenixUtil6.getInstance( ).cancoderPrintFaults(m_CANcoder, "FeederCANcoder");

    m_rollerMotor.clearStickyFaults( );
    m_rotaryMotor.clearStickyFaults( );
    m_CANcoder.clearStickyFaults( );
  }

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  public void setRollerMode(FDRollerMode mode)
  {
    double output = 0.0;

    if (mode == FDRollerMode.HOLD)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotor.get( )));
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
        case SCORE :
          output = kRollerSpeedScore;
          break;
        case HANDOFF :
          output = kRollerSpeedHandoff;
          break;
      }
      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_rollerMotor.set(output);
    }
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  private void moveRotaryWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > FDConsts.kRotaryAngleMin))
      newMode = FDConsts.RotaryMode.INBOARD;
    else if ((axisValue > 0.0) && (m_currentDegrees < FDConsts.kRotaryAngleMax))
      newMode = FDConsts.RotaryMode.OUTBOARD;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_rotaryMode)
    {
      m_rotaryMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_rotaryMode, getFeederPosition( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_rotaryMotor.setControl(m_requestVolts.withOutput(axisValue * kRotaryManualVolts));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement
   * 
   * @param newAngle
   *          rotation to move
   * @param holdPosition
   *          hold previous position if true
   */
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
        m_rotaryMotor.setControl(m_requestMMVolts.withPosition(targetRotations));
        DataLogManager
            .log(String.format("%s: Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ), m_currentDegrees,
                m_targetDegrees, Conversions.degreesToInputRotations(m_currentDegrees, kRotaryGearRatio), targetRotations));
      }
      else
        DataLogManager.log(String.format("%s: Position move target %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetDegrees, FDConsts.kRotaryAngleMin, FDConsts.kRotaryAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - target %s degrees", getSubsystem( ), m_targetDegrees));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  public void moveToPositionExecute( )
  {
    m_rotaryMotor
        .setControl(m_requestMMVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio)));
  }

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @return true when command has completed
   */
  public boolean moveToPositionIsFinished( )
  {
    boolean timedOut = m_safetyTimer.hasElapsed(kMMSafetyTimeout);
    double error = m_targetDegrees - m_currentDegrees;

    if (m_withinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_moveIsFinished)
        DataLogManager.log(String.format("%s: Position move finished - Current degrees: %.1f (error %.1f) - Time: %.3f sec %s",
            getSubsystem( ), m_currentDegrees, error, m_safetyTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_moveIsFinished = true;
    }

    return m_moveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToPositionEnd( )
  {
    m_safetyTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set feeder rotary motor to stopped
   * 
   */
  private void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: Rotary motor now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  /****************************************************************************
   * 
   * Get feeder rotary rotations
   * 
   * @return feeder rotary rotations
   */
  private double getRotaryRotations( )
  {
    return (m_fdRotaryValid) ? m_rotaryPosition.refresh( ).getValue( ) : 0.0;
  }

  /****************************************************************************
   * 
   * Get feeder CANcoder rotations
   * 
   * @return feeder rotary CANcoder rotations
   */
  private double getCANcoderRotations( )
  {
    double ccRotations = (m_fdCCValid) ? m_ccPosition.refresh( ).getValue( ) : 0.0;
    ccRotations -= (Robot.isReal( )) ? 0.0 : 0.0; // 0.359130859;
    return ccRotations;
  }

  /****************************************************************************
   * 
   * Validate requested feeder move
   * 
   * @param degrees
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double degrees)
  {
    return (degrees >= FDConsts.kRotaryAngleMin) && (degrees <= FDConsts.kRotaryAngleMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current feeder position
   * 
   * @return current feeder rotary angle
   */
  public double getFeederPosition( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return feeder angle for amp scoring state
   * 
   * @return amp scoring state angle
   */
  public double getFeederAmp( )
  {
    return FDConsts.kRotaryAngleAmp;
  }

  /****************************************************************************
   * 
   * Return feeder angle for climb state
   * 
   * @return climb state angle
   */
  public double getFeederClimb( )
  {
    return FDConsts.kRotaryAngleClimb;
  }

  /****************************************************************************
   * 
   * Return feeder angle for handoff state
   * 
   * @return handoff state angle
   */
  public double getFeederHandoff( )
  {
    return FDConsts.kRotaryAngleHandoff;
  }

  /****************************************************************************
   * 
   * Return feeder note sensor state
   * 
   * @return true if note detected in feeder
   */
  public boolean isNoteDetected( )
  {
    return m_noteDetected;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs rotary motor
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(                    // Command that runs continuously
        ( ) -> moveRotaryWithJoystick(axis),  // Lambda method to call
        this                                  // Subsystem required
    )                                         //
        .withName("FeederMoveWithJoystick");
  }

}
