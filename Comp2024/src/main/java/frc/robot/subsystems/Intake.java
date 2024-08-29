//
// Intake Subystem - takes in Notes and delivers them to the other subsystems
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Intake subsystem to control the intake roller and rotary mechanism and provide command factories
 */
public class Intake extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName        = "Intake";
  private static final boolean kRollerMotorInvert    = false;     // Motor direction for positive input

  private static final double  kRollerSpeedAcquire   = 0.5;
  private static final double  kRollerSpeedExpel     = -0.4;
  private static final double  kRollerSpeedToShooter = -0.4;      // TODO: speed this up to get closer to shooter speed
  private static final double  kRollerSpeedToFeeder  = -0.4;

  private static final double  kRotaryGearRatio      = 30.83;
  private static final double  kRotaryLengthMeters   = 0.3;       // Simulation
  private static final double  kRotaryWeightKg       = 4.0;       // Simulation
  private static final double  kRotaryManualVolts    = 3.5;       // Motor voltage during manual operation (joystick)

  /** Rotary manual move parameters */
  private enum RotaryMode
  {
    INIT,    // Initialize rotary
    INBOARD, // Rotary moving into the robot
    STOPPED, // Rotary stop and hold position
    OUTBOARD // Rotary moving out of the robot
  }

  // Rotary constants
  private static final double        kToleranceDegrees     = 4.0;      // PID tolerance in degrees
  private static final double        kMMMoveTimeout        = 1.5;      // Seconds allowed for a Motion Magic movement (TODO: TUNE ME)

  // Rotary angles - Motion Magic move parameters - TODO: Tune these angles!
  private static final double        kRotaryAngleRetracted = -97.5;
  private static final double        kRotaryAngleHandoff   = -49.9;
  private static final double        kRotaryAngleDeployed  = 99.4;

  private static final double        kRotaryAngleMin       = -99.0;
  private static final double        kRotaryAngleMax       = 101.4;

  // Device objects
  private final WPI_TalonSRX         m_rollerMotor         = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX              m_rotaryMotor         = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder             m_CANcoder            = new CANcoder(Ports.kCANID_IntakeCANcoder);
  private final DigitalInput         m_noteInIntake        = new DigitalInput(Ports.kDIO0_NoteInIntake);

  // Simulation objects
  private final TalonFXSimState      m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState     m_CANcoderSim         = m_CANcoder.getSimState( );
  private final SingleJointedArmSim  m_armSim              = new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d          m_rotaryMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d  m_mechLigament        = m_rotaryMech.getRoot("Rotary", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // Status signals
  private final StatusSignal<Double> m_rotaryPosition;  // Default 50Hz (20ms)
  private final StatusSignal<Double> m_ccPosition;      // Default 100Hz (10ms)

  // Declare module variables

  // Roller variables
  private boolean                    m_rollerValid;        // Health indicator for motor 
  private Debouncer                  m_noteDebouncer       = new Debouncer(0.030, DebounceType.kBoth);
  private boolean                    m_noteDetected;       // Detection state of note in rollers

  // Rotary variables
  private boolean                    m_rotaryValid;        // Health indicator for motor 
  private boolean                    m_canCoderValid;      // Health indicator for CANcoder 
  private double                     m_currentDegrees      = 0.0; // Current angle in degrees
  private double                     m_targetDegrees       = 0.0; // Target angle in degrees
  private double                     m_ccDegrees           = 0.0; // CANcoder angle in degrees

  // Manual mode config parameters
  private VoltageOut                 m_requestVolts        = new VoltageOut(0);
  private RotaryMode                 m_rotaryMode          = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage         m_mmRequestVolts      = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                  m_mmWithinTolerance   = new Debouncer(0.060, DebounceType.kRising);
  private Timer                      m_mmMoveTimer         = new Timer( ); // Safety timer for movements
  private boolean                    m_mmMoveIsFinished;   // Movement has completed (within tolerance)

  // Shuffleboard objects
  private ShuffleboardTab            m_subsystemTab        = Shuffleboard.getTab(kSubsystemName);
  private ShuffleboardLayout         m_rollerList          =
      m_subsystemTab.getLayout("Roller", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  private GenericEntry               m_rollValidEntry      = m_rollerList.add("rollValid", false).getEntry( );
  private GenericEntry               m_rollSpeedEntry      = m_rollerList.add("rollSpeed", 0.0).getEntry( );
  private GenericEntry               m_rollSupCurEntry     = m_rollerList.add("rollSupCur", 0.0).getEntry( );
  // private GenericEntry                      m_rollStatCurEntry    = m_rollerList.add("rollStatCur", 0.0).getEntry( );

  private ShuffleboardLayout         m_rotaryList          =
      m_subsystemTab.getLayout("Rotary", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3);
  private GenericEntry               m_rotValidEntry       = m_rotaryList.add("rotValid", false).getEntry( );
  private GenericEntry               m_rotDegreesEntry     = m_rotaryList.add("rotDegrees", 0.0).getEntry( );
  private GenericEntry               m_rotCLoopErrorEntry  = m_rotaryList.add("rotCLoopError", 0.0).getEntry( );

  private ShuffleboardLayout         m_statusList          =
      m_subsystemTab.getLayout("Status", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 3);
  private GenericEntry               m_ccValidEntry        = m_statusList.add("ccValid", false).getEntry( );
  private GenericEntry               m_ccDegreesEntry      = m_statusList.add("ccDegrees", 0.0).getEntry( );
  private GenericEntry               m_targetDegreesEntry  = m_statusList.add("targetDegrees", 0.0).getEntry( );
  private GenericEntry               m_noteDetectedEntry   = m_statusList.add("noteInDetected", false).getEntry( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Intake( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    Robot.timeMarker(getName( ) + ": constructor start");

    // Roller motor init
    m_rollerValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, kSubsystemName + "Roller",
        CTREConfigs5.intakeRollerConfig( ));
    m_rollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");
    m_rollValidEntry.setBoolean(m_rollerValid);

    // Rotary motor and CANcoder init
    m_rotaryValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, kSubsystemName + "Rotary",
        CTREConfigs6.intakeRotaryFXConfig(Units.degreesToRotations(kRotaryAngleMin), Units.degreesToRotations(kRotaryAngleMax),
            Ports.kCANID_IntakeCANcoder, kRotaryGearRatio));
    m_canCoderValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, kSubsystemName + "Rotary",
        CTREConfigs6.intakeRotaryCancoderConfig( ));
    m_rotValidEntry.setBoolean(m_rotaryValid);
    m_ccValidEntry.setBoolean(m_canCoderValid);

    m_rotaryPosition = m_rotaryMotor.getPosition( );
    m_ccPosition = m_CANcoder.getAbsolutePosition( ).waitForUpdate(10.0, false);

    Double ccRotations = (m_canCoderValid) ? m_ccPosition.refresh( ).getValue( ) : 0.0;
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_rotaryValid)
      m_rotaryMotor.setPosition(ccRotations);

    // Simulation object initialization
    m_rotarySim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANcoderSim.Orientation = ChassisReference.Clockwise_Positive;

    // Status signals
    m_rotaryPosition.setUpdateFrequency(50);

    StatusSignal<Double> m_rotarySupplyCur = m_rotaryMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Double> m_rotaryStatorCur = m_rotaryMotor.getStatorCurrent( ); // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(10, m_rotarySupplyCur, m_rotaryStatorCur);

    DataLogManager.log(
        String.format("%s: Update (Hz) rotaryPosition: %.1f rotarySupplyCur: %.1f rotaryStatorCur: %.1f canCoderPosition: %.1f",
            getSubsystem( ), m_rotaryPosition.getAppliedUpdateFrequency( ), m_rotarySupplyCur.getAppliedUpdateFrequency( ),
            m_rotaryStatorCur.getAppliedUpdateFrequency( ), m_ccPosition.getAppliedUpdateFrequency( )));

    initDashboard( );
    initialize( );

    Robot.timeMarker(getName( ) + ": constructor end");
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    BaseStatusSignal.refreshAll(m_rotaryPosition, m_ccPosition);
    m_currentDegrees = Units.rotationsToDegrees((m_rotaryValid) ? m_rotaryPosition.getValue( ) : 0.0);
    m_ccDegrees = Units.rotationsToDegrees((m_canCoderValid) ? m_ccPosition.getValue( ) : 0.0);
    m_noteDetected = m_noteDebouncer.calculate(m_noteInIntake.get( ));

    // Update dashboard
    m_rollSpeedEntry.setDouble(m_rollerMotor.get( ));
    m_rollSupCurEntry.setDouble(m_rollerMotor.getSupplyCurrent( ));

    m_ccDegreesEntry.setDouble(m_ccDegrees);
    m_rotDegreesEntry.setDouble(m_currentDegrees);
    m_noteDetectedEntry.setBoolean(m_noteDetected);
    m_targetDegreesEntry.setDouble(m_targetDegrees);
    m_rotCLoopErrorEntry.setDouble(m_targetDegrees - m_currentDegrees);
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

    m_mechLigament.setAngle(-m_currentDegrees);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets
    m_subsystemTab.add("INRotaryMech", m_rotaryMech).withPosition(0, 2);

    // Shuffleboard layout
    ShuffleboardLayout cmdList = m_subsystemTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 3)
        .withProperties(Map.of("Label position", "HIDDEN"));
    cmdList.add("InRollStop", getMoveToPositionCommand(INRollerMode.STOP, this::getCurrentPosition));
    cmdList.add("InRollAcquire", getMoveToPositionCommand(INRollerMode.ACQUIRE, this::getCurrentPosition));
    cmdList.add("InRollExpel", getMoveToPositionCommand(INRollerMode.EXPEL, this::getCurrentPosition));
    cmdList.add("InRollShoot", getMoveToPositionCommand(INRollerMode.SHOOT, this::getCurrentPosition));
    cmdList.add("InRollHold", getMoveToPositionCommand(INRollerMode.HOLD, this::getCurrentPosition));

    cmdList.add("InRotDeploy", getMoveToPositionCommand(INRollerMode.HOLD, this::getIntakeDeployed));
    cmdList.add("InRotRetract", getMoveToPositionCommand(INRollerMode.HOLD, this::getIntakeRetracted));
    cmdList.add("InRotHandoff", getMoveToPositionCommand(INRollerMode.HOLD, this::getIntakeHandoff));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    setRollerMode(INRollerMode.STOP);
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
    PhoenixUtil5.getInstance( ).talonSRXPrintFaults(m_rollerMotor, kSubsystemName + "Roller");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rotaryMotor, kSubsystemName + "Rotary");
    PhoenixUtil6.getInstance( ).cancoderPrintFaults(m_CANcoder, kSubsystemName + "CANcoder");

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
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_rotaryMode,
          getCurrentPosition( ), ((rangeLimited) ? " - RANGE LIMITED" : "")));
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
  public void moveToPositionInit(INRollerMode mode, double newAngle, boolean holdPosition)
  {
    setRollerMode(mode);
    m_mmMoveTimer.restart( );

    if (holdPosition)
      newAngle = getCurrentPosition( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Units.degreesToRotations(m_targetDegrees);
        m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Units.degreesToRotations(m_currentDegrees), targetRotations));
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
  {}

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

    m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_targetDegrees)));

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
  private void setRollerMode(INRollerMode mode)
  {
    double output = 0.0;

    if (mode == INRollerMode.HOLD)
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
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
        case SHOOT :
          output = kRollerSpeedToShooter;
          break;
        case HANDOFF :
          output = kRollerSpeedToFeeder;
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
  public double getCurrentPosition( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return intake angle for retracted state
   * 
   * @return retracted state angle
   */
  public double getIntakeRetracted( )
  {
    return kRotaryAngleRetracted;
  }

  /****************************************************************************
   * 
   * Return intake angle for handoff state
   * 
   * @return handoff state angle
   */
  public double getIntakeHandoff( )
  {
    return kRotaryAngleHandoff;
  }

  /****************************************************************************
   * 
   * Return intake angle for deployed state
   * 
   * @return deployed state angle
   */
  public double getIntakeDeployed( )
  {
    return kRotaryAngleDeployed;
  }

  /****************************************************************************
   * 
   * Return note sensor state
   * 
   * @return true if note detected
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
        .withName(kSubsystemName + "MoveWithJoystick");
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
  private Command getMMPositionCommand(INRollerMode mode, DoubleSupplier position, boolean hold)
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
  public Command getMoveToPositionCommand(INRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, false).withName(kSubsystemName + "MMMoveToPosition");
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
  public Command getHoldPositionCommand(INRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, true).withName(kSubsystemName + "MMHoldPosition");
  }

}
