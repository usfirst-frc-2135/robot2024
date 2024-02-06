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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.INConsts.RotaryMode;
import frc.robot.Constants.Ports;
<<<<<<< Updated upstream
=======
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
>>>>>>> Stashed changes
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil5;
import frc.robot.lib.util.PhoenixUtil6;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{

<<<<<<< Updated upstream
  public static final double     kRotaryGearRatio   = 27.41;
=======
  public static final class INConsts
  {
    public static final double        kGearRatio      = 27.41;

    public static final InvertedValue kInvertMotor    = InvertedValue.CounterClockwise_Positive; // Motor direction for positive input
    public static final boolean       kInvertCANCoder = true;
  }
>>>>>>> Stashed changes

  // Motion Magic config parameters

  // Member objects
<<<<<<< Updated upstream
  private final WPI_TalonSRX     m_rollerMotor      = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX          m_rotaryMotor      = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder         m_CANCoder         = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private final DigitalInput     m_noteInIntake     = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState  m_rotarySim        = m_rotaryMotor.getSimState( );
  private final CANcoderSimState m_CANCoderSim      = m_CANCoder.getSimState( );
=======
  private final WPI_TalonSRX     m_intakeRoller           = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX          m_intakeRotary           = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder         m_CANCoder               = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private final DigitalInput     m_noteInIntake           = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState  m_motorSim               = m_intakeRotary.getSimState( );
  private final CANcoderSimState m_CANCoderSim            = m_CANCoder.getSimState( );
>>>>>>> Stashed changes

  // Declare module variables

  // Roller variables

  // Rotary variables
  private boolean                m_rotaryValid;     // Health indicator for Falcon 
  private boolean                m_ccValid;         // Health indicator for CANCoder 
<<<<<<< Updated upstream
  private boolean                m_calibrated       = true;
  private boolean                m_debug            = true;

  private RotaryMode             m_rotaryMode       = RotaryMode.INIT;     // Manual movement mode with joysticks

  private static double          m_currentDegrees   = 0.0; // Current angle in degrees
  private double                 m_targetDegrees    = 0.0; // Target angle in degrees
=======
  private boolean                m_calibrated             = true;
  private boolean                m_debug                  = true;

  private INMode                 m_mode                   = INMode.INTAKEROTARY_INIT;     // Manual movement mode with joysticks

  private static double          m_currentDegrees         = 0.0; // Current angle in degrees
  private double                 m_targetDegrees          = 0.0; // Target angle in degrees
>>>>>>> Stashed changes

  private boolean                m_moveIsFinished;  // Movement has completed (within tolerance)

<<<<<<< Updated upstream
  private VoltageOut             m_requestVolts     = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage     m_requestMMVolts   = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                 m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                  m_safetyTimer      = new Timer( ); // Safety timer for movements
  private StatusSignal<Double>   m_rotaryPosition   = m_rotaryMotor.getRotorPosition( );
  private StatusSignal<Double>   m_rotaryVelocity   = m_rotaryMotor.getRotorVelocity( );
  private StatusSignal<Double>   m_rotaryCLoopError = m_rotaryMotor.getClosedLoopError( );
  private StatusSignal<Double>   m_rotarySupplyCur  = m_rotaryMotor.getSupplyCurrent( );
  private StatusSignal<Double>   m_rotaryStatorCur  = m_rotaryMotor.getStatorCurrent( );

  private boolean                m_intakeValid;
=======
  private VoltageOut             m_requestVolts           = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage     m_requestMMVolts         = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                 m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                  m_safetyTimer            = new Timer( ); // Safety timer for movements
  private StatusSignal<Double>   m_intakeRotaryPosition   = m_intakeRotary.getRotorPosition( );
  private StatusSignal<Double>   m_intakeRotaryVelocity   = m_intakeRotary.getRotorVelocity( );
  private StatusSignal<Double>   m_intakeRotaryCLoopError = m_intakeRotary.getClosedLoopError( );
  private StatusSignal<Double>   m_intakeRotarySupplyCur  = m_intakeRotary.getSupplyCurrent( );
  private StatusSignal<Double>   m_intakeRotaryStatorCur  = m_intakeRotary.getStatorCurrent( );
  private StatusSignal<Double>   m_ccPosition             = m_CANCoder.getAbsolutePosition( );
>>>>>>> Stashed changes

  // Manual config parameters

  //Devices and simulation objs

  // Constructor
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    m_intakeValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Intake Roller");
    SmartDashboard.putBoolean("HL_validIN", m_intakeValid);
    // TODO needs to be initialized 

    intakeTalonInitialize(m_rollerMotor, INConsts.kInvertMotor);
    initialize( );

    m_rotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Intake Rotary", CTREConfigs6.intakeRotaryFXConfig( ));
    m_ccValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "Intake Rotary", CTREConfigs6.intakeRotaryCancoderConfig( ));
<<<<<<< Updated upstream
=======

    if (Robot.isReal( ))
      m_currentDegrees = getCANCoderDegrees( );
    m_intakeRotary.setPosition(Conversions.degreesToInputRotations(m_currentDegrees, INConsts.kGearRatio));
    DataLogManager.log(String.format("%s: CANCoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

    m_motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANCoderSim.Orientation = ChassisReference.Clockwise_Positive;

    m_intakeRotaryPosition.setUpdateFrequency(50);
    if (m_debug)
    {
      m_intakeRotaryVelocity.setUpdateFrequency(50);
      m_intakeRotaryCLoopError.setUpdateFrequency(50);
      m_intakeRotarySupplyCur.setUpdateFrequency(50);
      m_intakeRotaryStatorCur.setUpdateFrequency(50);
    }
    m_ccPosition.setUpdateFrequency(50);

    initSmartDashboard( );
    initialize( );
>>>>>>> Stashed changes
  }

  @Override
  public void periodic( )
  {
    double currentDraw = m_rollerMotor.getStatorCurrent( );
    SmartDashboard.putNumber("INRoller_currentDraw", currentDraw);
    // This method will be called once per scheduler run

    m_currentDegrees = getTalonFXDegrees( );
    SmartDashboard.putNumber("IN_curDegrees", m_currentDegrees);
    SmartDashboard.putNumber("IN_targetDegrees", m_targetDegrees);
    SmartDashboard.putNumber("IN_CCDegrees", getCANCoderDegrees( ));
    SmartDashboard.putNumber("IN_totalFF", m_totalArbFeedForward);
    if (m_debug)
    {
      SmartDashboard.putNumber("IN_velocity", m_motorVelocity.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_curError", m_motorCLoopError.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_supplyCur", m_motorSupplyCur.refresh( ).getValue( ));
      SmartDashboard.putNumber("IN_statorCur", m_motorStatorCur.refresh( ).getValue( ));
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_rotarySim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANCoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));

  }

  // Initialize dashboard widgets

  private void initSmartDashboard( )
  {}

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    setRotaryStopped( );

    //m_currentDegrees = getTalonFXDegrees( );
    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getRotaryPosition( )
  {
    return m_currentDegrees;
  }

<<<<<<< Updated upstream
  public void setRotaryStopped( )
=======
  private double getCANCoderDegrees( )
  {
    return ((INConsts.kInvertCANCoder) ? -1.0 : 1.0)
        * Conversions.rotationsToOutputDegrees(m_ccPosition.refresh( ).getValue( ), 1.0);
  }

  private double getTalonFXDegrees( )
  {
    return Conversions.rotationsToOutputDegrees(m_motorPosition.refresh( ).getValue( ), INConsts.kGearRatio);
  }

  public void setStopped( )
>>>>>>> Stashed changes
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveRotaryWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.STOPPED;
  }

  private void intakeTalonInitialize(WPI_TalonSRX motor, boolean inverted)
  {
    motor.setInverted(inverted);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(motor, "setInverted");
  }

  public void setIntakeRollerSpeed(INRollerMode mode)
  {
    final String strName;
    double output = 0.0;

    switch (mode)
    {
      default :
      case ROLLER_STOP :
        strName = "STOP";
        output = 0.0;
        break;
      case ROLLER_ACQUIRE :
        strName = "ACQUIRE";
        output = INConsts.kIntakeRollerSpeedAcquire;
        break;
      case ROLLER_EXPEL :
        strName = "EXPEL";
        output = INConsts.kIntakeRollerSpeedExpel;
        break;
    }
    DataLogManager.log(String.format("%s: Mode is no - %s", getSubsystem( ), strName));
    m_rollerMotor.set(output);
  }

}
