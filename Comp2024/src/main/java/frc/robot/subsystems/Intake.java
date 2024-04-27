//
// Intake Subystem - takes in Notes and delivers them to the other subsystems
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.Constants.INConsts.RotaryMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Intake subsystem class
 */
public class Intake extends SubsystemBase
{
  // Constants
  private static final boolean      kRollerMotorInvert    = false;     // Motor direction for positive input

  private static final double       kRollerSpeedAcquire   = 0.5;
  private static final double       kRollerSpeedExpel     = -0.4;
  private static final double       kRollerSpeedToShooter = -0.4;
  private static final double       kRollerSpeedToFeeder  = -0.4;

  private static final double       kLigament2dOffset     = 90.0;      // Offset from mechanism root for ligament
  private static final double       kRotaryGearRatio      = 30.83;
  private static final double       kRotaryLengthMeters   = 0.3;
  private static final double       kRotaryWeightKg       = 4.0;
  private static final double       kRotaryManualVolts    = 3.5;      // Motor voltage during manual operation (joystick)

  // Rotary constants
  private static final double       kToleranceDegrees     = 4.0;      // PID tolerance in degrees
  private static final double       kMMSafetyTimeout      = 2.0;      // Seconds allowed for a Motion Magic movement (TODO: TUNE ME)

  // Device and simulation objects
  private static final WPI_TalonSRX m_rollerMotor         = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private static final TalonFX      m_rotaryMotor         = new TalonFX(Ports.kCANID_IntakeRotary);
  private static final CANcoder     m_CANcoder            = new CANcoder(Ports.kCANID_IntakeCANcoder);
  private static final DigitalInput m_noteInIntake        = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState     m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANcoderSim         = m_CANcoder.getSimState( );
  private final SingleJointedArmSim m_armSim              = new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d     m_mechRoot            = m_rotaryMech.getRoot("Rotary", 0.5, 0.5);
  private final MechanismLigament2d m_mechLigament        =
      m_mechRoot.append(new MechanismLigament2d("intake", 0.5, kLigament2dOffset, 6, new Color8Bit(Color.kPurple)));

  // Declare module variables

  // Roller variables
  private boolean                   m_inRollerValid;      // Health indicator for motor 

  // Rotary variables
  private boolean                   m_inRotaryValid;      // Health indicator for motor 
  private boolean                   m_inCCValid;          // Health indicator for CANcoder 
  private boolean                   m_debug               = true;
  private double                    m_currentDegrees      = 0.0; // Current angle in degrees
  private double                    m_targetDegrees       = 0.0; // Target angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts        = new VoltageOut(0);
  private RotaryMode                m_rotaryMode          = RotaryMode.INIT;     // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_requestMMVolts      = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                 m_withinTolerance     = new Debouncer(0.060, DebounceType.kRising);
  private Debouncer                 m_noteDetected        = new Debouncer(0.030, DebounceType.kBoth);
  private Timer                     m_safetyTimer         = new Timer( ); // Safety timer for movements
  private boolean                   m_moveIsFinished;     // Movement has completed (within tolerance)

  // Status signals
  private StatusSignal<Double>      m_rotaryPosition      = m_rotaryMotor.getRotorPosition( );    // Not used in MM - uses CANcoder remote sensor
  private StatusSignal<Double>      m_rotaryCLoopError    = m_rotaryMotor.getClosedLoopError( );
  private StatusSignal<Double>      m_rotarySupplyCur     = m_rotaryMotor.getSupplyCurrent( );
  private StatusSignal<Double>      m_rotaryStatorCur     = m_rotaryMotor.getStatorCurrent( );
  private StatusSignal<Double>      m_ccPosition          = m_CANcoder.getAbsolutePosition( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");

    // Roller motor init
    m_inRollerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Intake Roller", CTREConfigs5.intakeRollerConfig( ));
    m_rollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");

    // Rotary motor and CANcoder init
    m_inRotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Intake Rotary", CTREConfigs6.intakeRotaryFXConfig( ));
    m_inCCValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, "Intake Rotary", CTREConfigs6.intakeRotaryCancoderConfig( ));

    Double ccRotations = getCANcoderRotations( );
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_inRotaryValid)
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

    double rollerCurrent = (m_inRollerValid) ? m_rollerMotor.getStatorCurrent( ) : 0.0;
    SmartDashboard.putNumber("IN_rollerCur", rollerCurrent);

    // CANcoder is the primary (remote) sensor for Motion Magic
    m_currentDegrees = Conversions.rotationsToOutputDegrees(getRotaryRotations( ), kRotaryGearRatio);
    SmartDashboard.putNumber("IN_ccDegrees", Units.rotationsToDegrees(getCANcoderRotations( )));
    SmartDashboard.putNumber("IN_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("IN_targetDegrees", m_targetDegrees);
    SmartDashboard.putBoolean("IN_noteInIntake", m_noteInIntake.get( ));
    if (m_debug && m_inRotaryValid)
    {
      BaseStatusSignal.refreshAll(m_rotaryCLoopError, m_rotarySupplyCur, m_rotaryStatorCur);
      SmartDashboard.putNumber("IN_curError", m_rotaryCLoopError.getValue( ));
      SmartDashboard.putNumber("IN_rotSupCur", m_rotarySupplyCur.getValue( ));
      SmartDashboard.putNumber("IN_rotStatCur", m_rotaryStatorCur.getValue( ));
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
    SmartDashboard.putBoolean("HL_INValidRoller", m_inRollerValid);
    SmartDashboard.putBoolean("HL_INValidRotary", m_inRotaryValid);
    SmartDashboard.putBoolean("HL_INValidCANcoder", m_inCCValid);

    SmartDashboard.putData("INRotaryMech", m_rotaryMech);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    setRollerMode(RollerMode.STOP);
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
    PhoenixUtil5.getInstance( ).talonSRXPrintFaults(m_rollerMotor, "IntakeRoller");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rotaryMotor, "IntakeRotary");
    PhoenixUtil6.getInstance( ).cancoderPrintFaults(m_CANcoder, "IntakeCANcoder");

    m_rollerMotor.clearStickyFaults( );
    m_rotaryMotor.clearStickyFaults( );
    m_CANcoder.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current intake position
   * 
   * @return current intake rotary angle
   */
  public double getIntakePosition( )
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
    return INConsts.kRotaryAngleRetracted;
  }

  /****************************************************************************
   * 
   * Return intake angle for handoff state
   * 
   * @return handoff state angle
   */
  public double getIntakeHandoff( )
  {
    return INConsts.kRotaryAngleHandoff;
  }

  /****************************************************************************
   * 
   * Return intake angle for deployed state
   * 
   * @return deployed state angle
   */
  public double getIntakeDeployed( )
  {
    return INConsts.kRotaryAngleDeployed;
  }

  /****************************************************************************
   * 
   * Return intake note sensor state
   * 
   * @return true if note detected in intake
   */
  public boolean isNoteDetected( )
  {
    return m_noteDetected.calculate(m_noteInIntake.get( ));
  }

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  public void setRollerMode(RollerMode mode)
  {
    double output = 0.0;

    if (mode == RollerMode.HOLD)
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

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  public void moveRotaryWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
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
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_rotaryMode, getIntakePosition( ),
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
      newAngle = getIntakePosition( );

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
        DataLogManager.log(String.format("%s: Position move %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]", getSubsystem( ),
            m_targetDegrees, INConsts.kRotaryAngleMin, INConsts.kRotaryAngleMax));
    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved -target %s degrees", getSubsystem( ), m_targetDegrees));
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
   * @return true when movement has completed
   */
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

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToPositionEnd( )
  {
    m_safetyTimer.stop( );
  }

  ///////////////////////// PRIVATE HELPERS ///////////////////////////////

  /****************************************************************************
   * 
   * Set intake rotary motor to stopped
   */
  private void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: Rotary motor now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  /****************************************************************************
   * 
   * Get intake rotary rotations
   * 
   * @return intake rotary rotations
   */
  private double getRotaryRotations( )
  {
    return (m_inRotaryValid) ? m_rotaryPosition.refresh( ).getValue( ) : 0.0;
  }

  /****************************************************************************
   * 
   * Get intake CANcoder rotations
   * 
   * @return intake rotary CANcoder rotations
   */
  private double getCANcoderRotations( )
  {
    double ccRotations = (m_inCCValid) ? m_ccPosition.refresh( ).getValue( ) : 0.0;
    ccRotations -= (Robot.isReal( )) ? 0.0 : 0.0; // 0.359130859;
    return ccRotations;
  }

  /****************************************************************************
   * 
   * Validate requested intake move
   * 
   * @param degrees
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double degrees)
  {
    return (degrees >= INConsts.kRotaryAngleMin) && (degrees <= INConsts.kRotaryAngleMax);
  }

}
