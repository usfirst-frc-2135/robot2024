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
import frc.robot.lib.util.CTREConfigs5;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil5;
import frc.robot.lib.util.PhoenixUtil6;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{

  public static final double     kRotaryGearRatio   = 27.41;

  // Motion Magic config parameters

  // Member objects
  private final WPI_TalonSRX     m_rollerMotor      = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX          m_rotaryMotor      = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder         m_CANCoder         = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private final DigitalInput     m_noteInIntake     = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState  m_rotarySim        = m_rotaryMotor.getSimState( );
  private final CANcoderSimState m_CANCoderSim      = m_CANCoder.getSimState( );

  // Declare module variables

  // Roller variables

  // Rotary variables
  private boolean                m_rotaryValid;     // Health indicator for Falcon 
  private boolean                m_ccValid;         // Health indicator for CANCoder 
  private boolean                m_calibrated       = true;
  private boolean                m_debug            = true;

  private RotaryMode             m_rotaryMode       = RotaryMode.INIT;     // Manual movement mode with joysticks

  private static double          m_currentDegrees   = 0.0; // Current angle in degrees
  private double                 m_targetDegrees    = 0.0; // Target angle in degrees

  private boolean                m_moveIsFinished;  // Movement has completed (within tolerance)

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

  // Manual config parameters

  //Devices and simulation objs

  // Constructor
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    m_intakeValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, "Intake Roller", CTREConfigs5.intakeRollerConfig( ));
    SmartDashboard.putBoolean("HL_validIN", m_intakeValid);
    // TODO needs to be initialized 

    intakeTalonInitialize(m_rollerMotor, INConsts.kInvertMotor);
    initialize( );

    m_rotaryValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, "Intake Rotary", CTREConfigs6.intakeRotaryFXConfig( ));
    m_ccValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "Intake Rotary", CTREConfigs6.intakeRotaryCancoderConfig( ));
  }

  @Override
  public void periodic( )
  {
    double currentDraw = m_rollerMotor.getStatorCurrent( );
    SmartDashboard.putNumber("INRoller_currentDraw", currentDraw);
    // This method will be called once per scheduler run

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

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getRotaryPosition( )
  {
    return m_currentDegrees;
  }

  public void setRotaryStopped( )
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
