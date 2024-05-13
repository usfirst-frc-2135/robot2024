//
// Feeder Subystem - takes in Notes and feeds them into the Amp and Trap
//
package frc.robot.subsystems;

import java.util.Map;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Feeder subsystem to control the feeder roller and rotary mechanism and provide command factories
 */
public class Feeder extends SubsystemBase
{

  // Constants
  private static final String  kFeederTab          = "Feeder";
  private static final boolean kRollerMotorInvert  = true;      // Motor direction for positive input

  private static final double  kRollerSpeedScore   = -0.5;
  private static final double  kRollerSpeedHandoff = -0.37;

  private static final double  kRotaryGearRatio    = 27.0;
  private static final double  kRotaryLengthMeters = 0.5;       // Simulation
  private static final double  kRotaryWeightKg     = 4.0;       // Simulation
  private static final double  kRotaryManualVolts  = 3.5;       // Motor voltage during manual operation (joystick)

  /** Rotary manual move parameters */
  private enum RotaryMode
  {
    INIT,    // Initialize feeder
    INBOARD, // Feeder Rotary moving into the feeder
    STOPPED, // Feeder Rotary stop and hold position
    OUTBOARD // Feeder Rotary moving out of the feeder
  }

  // Rotary constants
  private static final double       kToleranceDegrees     = 4.0;      // PID tolerance in degrees
  private static final double       kMMMoveTimeout        = 2.0;      // Seconds allowed for a Motion Magic movement (TODO: TUNE ME)

  // Rotary angles - Motion Magic move parameters - TODO: Tune these angles!
  public static final double        kRotaryAngleAmp       = -33.0;
  public static final double        kRotaryAngleClimb     = 60.0;
  public static final double        kRotaryAngleHandoff   = 88.75;

  public static final double        kRotaryAngleMin       = -61.89;
  public static final double        kRotaryAngleMax       = 90.0;

  // Device objects
  private static final WPI_TalonSRX m_rollerMotor         = new WPI_TalonSRX(Ports.kCANID_FeederRoller);
  private static final TalonFX      m_rotaryMotor         = new TalonFX(Ports.kCANID_FeederRotary);
  private static final CANcoder     m_CANcoder            = new CANcoder(Ports.kCANID_FeederCANcoder);
  private static final DigitalInput m_noteInFeeder        = new DigitalInput(Ports.kDIO1_NoteInFeeder);

  // Simulation objects
  private final TalonFXSimState     m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANcoderSim         = m_CANcoder.getSimState( );
  private final SingleJointedArmSim m_armSim              = new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d     m_mechRoot            = m_rotaryMech.getRoot("Rotary", 0.5, 0.5);
  private final MechanismLigament2d m_mechLigament        =
      m_mechRoot.append(new MechanismLigament2d("feeder", 0.5, 0.0, 6, new Color8Bit(Color.kBlue)));

  // Declare module variables

  // Roller variables
  private boolean                   m_rollerValid;        // Health indicator for motor 
  private Debouncer                 m_noteDebouncer       = new Debouncer(0.030, DebounceType.kBoth);
  private boolean                   m_noteDetected;       // Detection state of note in rollers

  // Rotary variables
  private boolean                   m_rotaryValid;        // Health indicator for motor 
  private boolean                   m_canCoderValid;      // Health indicator for CANcoder 
  private boolean                   m_debug               = true;
  private double                    m_currentDegrees      = 0.0; // Current angle in degrees
  private double                    m_targetDegrees       = 0.0; // Target angle in degrees
  private double                    m_ccDegrees           = 0.0; // CANcoder angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts        = new VoltageOut(0);
  private RotaryMode                m_rotaryMode          = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_mmRequestVolts      = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                 m_mmWithinTolerance   = new Debouncer(0.060, DebounceType.kRising);
  private Timer                     m_mmMoveTimer         = new Timer( ); // Safety timer for movements
  private boolean                   m_mmMoveIsFinished;   // Movement has completed (within tolerance)

  // Status signals
  private StatusSignal<Double>      m_rotaryPosition      = m_rotaryMotor.getRotorPosition( );    // Not used in MM - uses CANcoder remote sensor
  private StatusSignal<Double>      m_rotaryCLoopError    = m_rotaryMotor.getClosedLoopError( );
  private StatusSignal<Double>      m_rotarySupplyCur     = m_rotaryMotor.getSupplyCurrent( );
  private StatusSignal<Double>      m_rotaryStatorCur     = m_rotaryMotor.getStatorCurrent( );
  private StatusSignal<Double>      m_ccPosition          = m_CANcoder.getAbsolutePosition( );

  // Shuffleboard objects
  ShuffleboardTab                   m_feederTab           = Shuffleboard.getTab(kFeederTab);
  ShuffleboardLayout                m_rollerList          =
      m_feederTab.getLayout("Roller", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  GenericEntry                      m_rollValidEntry      = m_rollerList.add("rollValid", false).getEntry( );
  GenericEntry                      m_rollSpeedEntry      = m_rollerList.add("rollSpeed", 0.0).getEntry( );
  GenericEntry                      m_rollSupCurEntry     = m_rollerList.add("rollSupCur", 0.0).getEntry( );
  GenericEntry                      m_rollStatCurEntry    = m_rollerList.add("rollStatCur", 0.0).getEntry( );

  ShuffleboardLayout                m_rotaryList          =
      m_feederTab.getLayout("Rotary", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
  GenericEntry                      m_rotValidEntry       = m_rotaryList.add("rotValid", false).getEntry( );
  GenericEntry                      m_rotDegreesEntry     = m_rotaryList.add("rotDegrees", 0.0).getEntry( );
  GenericEntry                      m_rotCLoopErrorEntry  = m_rotaryList.add("rotCLoopError", 0.0).getEntry( );
  GenericEntry                      m_rotSupCurEntry      = m_rotaryList.add("rotSupCur", 0.0).getEntry( );
  GenericEntry                      m_rotStatCurEntry     = m_rotaryList.add("rotStatCur", 0.0).getEntry( );

  ShuffleboardLayout                m_statusList          =
      m_feederTab.getLayout("Status", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);
  GenericEntry                      m_ccValidEntry        = m_statusList.add("ccValid", false).getEntry( );
  GenericEntry                      m_ccDegreesEntry      = m_statusList.add("ccDegrees", 0.0).getEntry( );
  GenericEntry                      m_currentDegreesEntry = m_statusList.add("currentDegrees", 0.0).getEntry( );
  GenericEntry                      m_targetDegreesEntry  = m_statusList.add("targetDegrees", 0.0).getEntry( );
  GenericEntry                      m_noteInFeederEntry   = m_statusList.add("noteInFeeder", false).getEntry( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Feeder( )
  {
    setName("Feeder");
    setSubsystem("Feeder");

    // Roller motor init
    m_rollerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Feeder Roller", CTREConfigs5.feederRollerConfig( ));
    m_rollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");
    m_rollValidEntry.setBoolean(m_rollerValid);

    // Rotary motor and CANcoder init
    m_rotaryValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Feeder Rotary",
        CTREConfigs6.feederRotaryFXConfig(Units.degreesToRotations(kRotaryAngleMin), Units.degreesToRotations(kRotaryAngleMax)));
    m_canCoderValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, "Feeder Rotary", CTREConfigs6.feederRotaryCancoderConfig( ));
    m_rotValidEntry.setBoolean(m_rotaryValid);
    m_ccValidEntry.setBoolean(m_canCoderValid);

    Double ccRotations = getCANcoderRotations( );
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_rotaryValid)
      m_rotaryMotor.setPosition(Conversions.rotationsToInputRotations(ccRotations, kRotaryGearRatio)); // Not really used - CANcoder is remote sensor with absolute position

    // Simulation object initialization
    m_rotarySim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANcoderSim.Orientation = ChassisReference.Clockwise_Positive;

    m_rotaryPosition.setUpdateFrequency(50);
    if (m_debug)
      BaseStatusSignal.setUpdateFrequencyForAll(20, m_rotaryCLoopError, m_rotarySupplyCur, m_rotaryStatorCur);
    m_ccPosition.setUpdateFrequency(50);

    initDashboard( );
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

    m_currentDegrees = Conversions.rotationsToOutputDegrees(getRotaryRotations( ), kRotaryGearRatio);
    m_ccDegrees = Units.rotationsToDegrees(getCANcoderRotations( ));
    m_noteDetected = m_noteDebouncer.calculate(m_noteInFeeder.get( ));

    // Update dashboard
    m_rollSpeedEntry.setDouble(m_rollerMotor.get( ));
    m_rollSupCurEntry.setDouble(m_rollerMotor.getSupplyCurrent( ));
    m_rollStatCurEntry.setDouble(m_rollerMotor.getStatorCurrent( ));

    m_currentDegreesEntry.setDouble(m_currentDegrees);
    m_ccDegreesEntry.setDouble(m_ccDegrees);
    m_rotDegreesEntry.setDouble(m_currentDegrees);
    m_noteInFeederEntry.setBoolean(m_noteDetected);
    m_targetDegreesEntry.setDouble(m_targetDegrees);

    BaseStatusSignal.refreshAll(m_rotaryCLoopError, m_rotarySupplyCur, m_rotaryStatorCur);
    m_rotCurErrorEntry.setDouble(m_rotaryCLoopError.getValue( ));
    m_rotSupCurEntry.setDouble(m_rotarySupplyCur.getValue( ));
    m_rotStatCurEntry.setDouble(m_rotaryStatorCur.getValue( ));
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

    m_mechLigament.setAngle(m_currentDegrees);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets
    m_feederTab.add("FDRotaryMech", m_rotaryMech).withPosition(0, 3);

    // Shuffleboard layout
    ShuffleboardLayout cmdList = m_feederTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4)
        .withProperties(Map.of("Label position", "HIDDEN"));
    cmdList.add("FdRollStop", getMoveToPositionCommand(FDRollerMode.STOP, this::getFeederPosition));
    cmdList.add("FdRollScore", getMoveToPositionCommand(FDRollerMode.SCORE, this::getFeederPosition));
    cmdList.add("FdRollHandoff", getMoveToPositionCommand(FDRollerMode.HANDOFF, this::getFeederPosition));
    cmdList.add("FdRollHold", getMoveToPositionCommand(FDRollerMode.HOLD, this::getFeederPosition));

    cmdList.add("FdRotAmp", getMoveToPositionCommand(FDRollerMode.HOLD, this::getFeederAmp));
    cmdList.add("FdRotClimb", getMoveToPositionCommand(FDRollerMode.HOLD, this::getFeederClimb));
    cmdList.add("FdRotHandoff", getMoveToPositionCommand(FDRollerMode.HOLD, this::getFeederHandoff));
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

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

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

    if ((axisValue < 0.0) && (m_currentDegrees > kRotaryAngleMin))
      newMode = RotaryMode.INBOARD;
    else if ((axisValue > 0.0) && (m_currentDegrees < kRotaryAngleMax))
      newMode = RotaryMode.OUTBOARD;
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

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement and control rollers
   * 
   * @param mode
   *          roller mode to apply
   * @param newAngle
   *          rotation to move
   * @param holdPosition
   *          hold previous position if true
   */
  public void moveToPositionInit(FDRollerMode mode, double newAngle, boolean holdPosition)
  {
    setRollerMode(mode);
    m_mmMoveTimer.restart( );

    if (holdPosition)
      newAngle = getFeederPosition( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio);
        m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        DataLogManager
            .log(String.format("%s: MM Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ), m_currentDegrees,
                m_targetDegrees, Conversions.degreesToInputRotations(m_currentDegrees, kRotaryGearRatio), targetRotations));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetDegrees, kRotaryAngleMin, kRotaryAngleMax));
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved -target %s degrees", getSubsystem( ), m_targetDegrees));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  public void moveToPositionExecute( )
  {
    m_rotaryMotor
        .setControl(m_mmRequestVolts.withPosition(Conversions.degreesToInputRotations(m_targetDegrees, kRotaryGearRatio)));
  }

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @return true when movement has completed
   */
  public boolean moveToPositionIsFinished(boolean hold)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetDegrees - m_currentDegrees;

    if (hold)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_mmMoveIsFinished)
        DataLogManager.log(String.format("%s: MM Position move finished - Current degrees: %.1f (error %.1f) - Time: %.3f sec %s",
            getSubsystem( ), m_currentDegrees, error, m_mmMoveTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToPositionEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setRollerMode(FDRollerMode mode)
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

  /****************************************************************************
   * 
   * Set rotary motor to stopped
   */
  private void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: Rotary motor now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  /****************************************************************************
   * 
   * Get rotary rotations
   * 
   * @return rotary rotations
   */
  private double getRotaryRotations( )
  {
    return (m_rotaryValid) ? m_rotaryPosition.refresh( ).getValue( ) : 0.0;
  }

  /****************************************************************************
   * 
   * Get CANcoder rotations
   * 
   * @return rotary CANcoder rotations
   */
  private double getCANcoderRotations( )
  {
    return (m_canCoderValid) ? m_ccPosition.refresh( ).getValue( ) : 0.0;
  }

  /****************************************************************************
   * 
   * Validate requested move
   * 
   * @param degrees
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double degrees)
  {
    return (degrees >= kRotaryAngleMin) && (degrees <= kRotaryAngleMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current position
   * 
   * @return current rotary angle
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
    return kRotaryAngleAmp;
  }

  /****************************************************************************
   * 
   * Return feeder angle for climb state
   * 
   * @return climb state angle
   */
  public double getFeederClimb( )
  {
    return kRotaryAngleClimb;
  }

  /****************************************************************************
   * 
   * Return feeder angle for handoff state
   * 
   * @return handoff state angle
   */
  public double getFeederHandoff( )
  {
    return kRotaryAngleHandoff;
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

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance
   * @param hold
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs climber motors
   */
  private Command getMMPositionCommand(FDRollerMode mode, DoubleSupplier position, boolean hold)
  {
    return new FunctionalCommand(                                 // Command with all phases declared
        ( ) -> moveToPositionInit(mode, position.getAsDouble( ), hold), // Init method
        ( ) -> moveToPositionExecute( ),                          // Execute method
        interrupted -> moveToPositionEnd( ),                      // End method
        ( ) -> moveToPositionIsFinished(hold),                    // IsFinished method
        this                                                      // Subsytem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to position command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getMoveToPositionCommand(FDRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, false).withName("FeederMMMoveToPosition");
  }

  /****************************************************************************
   * 
   * Create motion magic hold position command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getHoldPositionCommand(FDRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, true).withName("FeederMMHoldPosition");
  }

}
