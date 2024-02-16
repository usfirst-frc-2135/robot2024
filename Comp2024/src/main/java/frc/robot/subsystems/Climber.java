//
// Climber Subystem - lifts the robot to hang onto the chain
//
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CLConsts.ClimberMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil6;

//
// Climber subsystem class
//
public class Climber extends SubsystemBase
{
  // Constants
  private final double              kLigament2dOffset    = 0.0;    // Offset from mechanism root for climber ligament
  public static final double        kGearRatio           = 8.57;   // Gear reduction
  public static final double        kDrumDiameterInches  = 1.375;  // Drum diameter in inches
  public static final double        kDrumRadiusMeters    = Units.inchesToMeters(kDrumDiameterInches) / 2;
  public static final double        kDrumCircumInches    = kDrumDiameterInches * Math.PI;             // Drum diameter in inches
  public static final double        kRolloutRatio        = kDrumCircumInches / kGearRatio; // inches per shaft rotation
  public static final double        kCalibrateSpeedVolts = -1.4;   // Motor voltage during calibration
  public static final double        kManualSpeedVolts    = 3.0;    // Motor voltage during manual operation (joystick)

  // Climber lengths
  public static final double        kLengthIn            = 0.25;   // By definition - Climber fully retracted
  public static final double        kLengthStop          = -0.25;  // Slightly off mechanical hard stop
  public static final double        kLengthOut           = 17.0;   // From Mech Design (floor, feet art 5" high), empirically checked
  public static final double        kLengthMin           = -0.5;   // Climber minimum allowable length (half inch less than stowed)
  public static final double        kLengthMax           = 18.25;  // Climber maximum allowable length (2" beyond high length)
  public static final double        kLengthClimber       = 1.0;

  public static final double        kToleranceInches     = 0.5;    // Climber PID tolerance in inches
  public static final double        kMMSafetyTimeout     = 2.0;    // Seconds allowed for a Motion Magic movement

  // Device and simulation objects
  private final TalonFX             m_climberL           = new TalonFX(Ports.kCANID_ClimberL);
  private final TalonFX             m_climberR           = new TalonFX(Ports.kCANID_ClimberR);

  private final TalonFXSimState     m_climberSim         = m_climberL.getSimState( );
  private final ElevatorSim         m_elevSim            =
      new ElevatorSim(DCMotor.getFalcon500(1), kGearRatio, 2.0, kDrumRadiusMeters, -kLengthMax, kLengthMax, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_climberMech        = new Mechanism2d(2, 2);
  private final MechanismRoot2d     m_mechRoot           = m_climberMech.getRoot("Climber", 1.0, 1.0);
  private final MechanismLigament2d m_mechLigament       =
      m_mechRoot.append(new MechanismLigament2d("climberr", kLengthClimber, kLigament2dOffset, 6, new Color8Bit(Color.kRed)));

  // Declare module variables
  private static boolean            m_isComp;

  private boolean                   m_motorValid;                   // Health indicator for Falcon 
  private boolean                   m_calibrated         = true;
  private boolean                   m_debug              = true;
  private double                    m_currentInches      = 0.0;    // Current length in inches
  private double                    m_targetInches       = 0.0;    // Target length in inches
  private int                       m_hardStopCounter    = 0;

  // Manual mode config parameters
  private VoltageOut                m_requestVolts       = new VoltageOut(0).withEnableFOC(false);
  private ClimberMode               m_mode               = ClimberMode.INIT;  // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_requestMMVolts     = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                 m_withinTolerance    = new Debouncer(0.060, DebounceType.kRising);
  private Timer                     m_safetyTimer        = new Timer( ); // Safety timer for movements
  private double                    m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity
  private boolean                   m_moveIsFinished;        // Movement has completed (within tolerance)

  private StatusSignal<Double>      m_motorPosition      = m_climberL.getRotorPosition( );
  private StatusSignal<Double>      m_motorVelocity      = m_climberL.getRotorVelocity( );
  private StatusSignal<Double>      m_motorCLoopError    = m_climberL.getClosedLoopError( );
  private StatusSignal<Double>      m_motorSupplyCur     = m_climberL.getSupplyCurrent( );
  private StatusSignal<Double>      m_motorStatorCur     = m_climberL.getStatorCurrent( );

  // Constructor

  public Climber(boolean isComp)
  {
    setName("Climber");
    setSubsystem("Climber");
    m_isComp = isComp;

    m_motorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_climberL, "ClimberL", CTREConfigs6.climberLengthFXConfig( ))
        && PhoenixUtil6.getInstance( ).talonFXInitialize6(m_climberR, "ClimberR", CTREConfigs6.climberLengthFXConfig( ));
    m_climberR.setControl(new Follower(m_climberL.getDeviceID( ), false));

    m_climberL.setPosition(Conversions.inchesToWinchRotations(m_currentInches, kRolloutRatio));
    DataLogManager.log(String.format("%s: CANCoder initial inches %.1f", getSubsystem( ), m_currentInches));

    // Simulation object initialization
    m_climberSim.Orientation = ChassisReference.CounterClockwise_Positive;

    m_motorPosition.setUpdateFrequency(50);
    if (m_debug)
    {
      m_motorVelocity.setUpdateFrequency(50);
      m_motorCLoopError.setUpdateFrequency(50);
      m_motorSupplyCur.setUpdateFrequency(50);
      m_motorStatorCur.setUpdateFrequency(50);
    }

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_currentInches = getCurrentInches( );
    if (m_currentInches < 0)
      setClimberToZero( );

    SmartDashboard.putNumber("CL_curInches", m_currentInches);
    SmartDashboard.putNumber("CL_targetInches", m_targetInches);
    SmartDashboard.putNumber("CL_curRotations", Conversions.inchesToWinchRotations(m_currentInches, kRolloutRatio));
    SmartDashboard.putNumber("CL_totalFF", m_totalArbFeedForward);
    if (m_debug)
    {
      SmartDashboard.putNumber("CL_velocity", m_motorVelocity.refresh( ).getValue( ));
      SmartDashboard.putNumber("CL_curError", m_motorCLoopError.refresh( ).getValue( ));
      SmartDashboard.putNumber("CL_supplyCur", m_motorSupplyCur.refresh( ).getValue( ));
      SmartDashboard.putNumber("CL_statorCur", m_motorStatorCur.refresh( ).getValue( ));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_climberSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_elevSim.setInput(m_climberSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elevSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_climberSim.setRawRotorPosition(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getPositionMeters( )), kRolloutRatio));
    m_climberSim.setRotorVelocity(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getVelocityMetersPerSecond( )), kRolloutRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevSim.getCurrentDrawAmps( )));

    m_mechLigament.setLength(kLengthClimber + Units.inchesToMeters(m_currentInches));
  }

  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets
    SmartDashboard.putBoolean("HL_validCL", m_motorValid);
    SmartDashboard.putNumber("CL_ArbFF", 0.0);
    SmartDashboard.putData("ClimberMech", m_climberMech);
  }

  public void initialize( )
  {
    setStopped( );
    m_calibrated = false;

    m_currentInches = 0.5; // Allow calibration routine to run for up to this length
    m_targetInches = m_currentInches;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Inches: %.1f", getSubsystem( ), m_targetInches));
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getLength( )
  {
    return m_currentInches;
  }

  private double getCurrentInches( )
  {
    return Conversions.rotationsToWinchInches(m_motorPosition.refresh( ).getValue( ), kRolloutRatio);
  }

  private boolean isMoveValid(double inches)
  {
    return (inches > kLengthMin) && (inches < kLengthMax);
  }

  private boolean isWithinTolerance(double targetInches)
  {
    return (Math.abs(targetInches - m_currentInches) < kToleranceInches);
  }

  public void resetPositionToZero( )
  {
    if (m_motorValid)
      m_climberL.setPosition(Conversions.inchesToWinchRotations(0, kRolloutRatio));
  }

  public void setStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_climberL.setControl(m_requestVolts.withOutput(0.0));
  }

  public void setMMPosition(double targetInches)
  {
    // y = mx + b, where 0 degrees is 0.0 climber and 90 degrees is 1/4 winch turn (the climber constant)
    m_climberL.setControl(m_requestMMVolts.withPosition(Conversions.inchesToWinchRotations(targetInches, kRolloutRatio))
        .withFeedForward(m_totalArbFeedForward));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveWithJoystick(double axisValue)
  {
    boolean rangeLimited = false;
    ClimberMode newMode = ClimberMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentInches > kLengthMin))
      newMode = ClimberMode.IN;
    else if ((axisValue > 0.0) && (m_currentInches < kLengthMax))
      newMode = ClimberMode.OUT;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: move %s %.1f in %s", getSubsystem( ), m_mode, getCurrentInches( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetInches = m_currentInches;

    m_climberL.setControl(m_requestVolts.withOutput(axisValue * kManualSpeedVolts));
  }

  ///////////////////////// MOTION MAGIC ///////////////////////////////////

  public void moveToPositionInit(double newLength, boolean holdPosition)
  {
    m_safetyTimer.restart( );
    m_hardStopCounter = 0;

    if (holdPosition)
      newLength = getCurrentInches( );

    // Decide if a new position request
    if (holdPosition || newLength != m_targetInches || !isWithinTolerance(newLength))
    {
      // Validate the position request
      if (isMoveValid(newLength))
      {
        m_targetInches = newLength;
        m_moveIsFinished = false;
        m_withinTolerance.calculate(false); // Reset the debounce filter

        setMMPosition(m_targetInches);

        DataLogManager.log(String.format("%s: Position move: %.1f -> %.1f inches (%.1f -> %.1f)", getSubsystem( ),
            m_currentInches, m_targetInches, Conversions.inchesToWinchRotations(m_currentInches, kRolloutRatio),
            Conversions.inchesToWinchRotations(m_targetInches, kRolloutRatio)));
      }
      else
        DataLogManager.log(String.format("%s: Position move %.1f inches is OUT OF RANGE! [%.1f, %.1f]", getSubsystem( ),
            m_targetInches, kLengthMin, kLengthMax));

    }
    else
    {
      m_moveIsFinished = true;
      DataLogManager.log(String.format("%s: Position already achieved - %s", getSubsystem( ), m_targetInches));
    }
  }

  public void moveToPositionExecute( )
  {
    if (m_calibrated)
      setMMPosition(m_targetInches);
  }

  public boolean moveToPositionIsFinished( )
  {
    boolean timedOut = m_safetyTimer.hasElapsed(kMMSafetyTimeout);
    double error = m_targetInches - m_currentInches;
    boolean hittingHardStop = (m_targetInches <= 0.0) && (m_currentInches <= 1.0) && (m_hardStopCounter++ >= 10);

    if (m_withinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut || hittingHardStop)
    {
      if (hittingHardStop)
        DataLogManager.log(String.format("%s - HITTINGHARDSTOP: %s", getSubsystem( ), hittingHardStop));
      if (!m_moveIsFinished)
        DataLogManager.log(String.format("%s: Position move finished - Current inches: %.1f (error %.1f) - Time: %.3f %s",
            getSubsystem( ), m_currentInches, error, m_safetyTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_moveIsFinished = true;
    }

    return m_moveIsFinished;
  }

  public void moveToPositionEnd( )
  {
    m_safetyTimer.stop( );
  }

  ///////////////////////// CALIBRATION ///////////////////////////////////

  public void moveToCalibrate( )
  {
    if (m_motorValid)
      m_climberL.setControl(m_requestVolts.withOutput(kCalibrateSpeedVolts));
  }

  public void endCalibration( )
  {
    setClimberToZero( );
    m_targetInches = m_currentInches;
    m_calibrated = true;
    SmartDashboard.putBoolean("CL_calibrated", m_calibrated);
  }

  private void setClimberToZero( )
  {
    m_currentInches = 0.0;
    resetPositionToZero( );
  }

}
