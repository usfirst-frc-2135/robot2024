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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil6;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{

  public static final class INConsts
  {
    public static final double kGearRatio = 27.41;
  }

  public enum INMode
  {
    // Add these contants for the roller
    INTAKEROTARY_INIT,    // Initialize intake
    INTAKEROTARY_DOWN,    // IntakeRotar moving down
    INTAKEROTARY_STOPPED, // IntakeRotar stop and hold position
    INTAKEROTARY_UP       // IntakeRotar moving up
  }
  // Motion Magic config parameters

  // Member objects
  private final WPI_TalonSRX     m_intakeRoller    = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX          m_intakeRotary    = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder         m_CANCoder        = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private final DigitalInput     m_noteInIntake    = new DigitalInput(Ports.kDIO0_NoteInIntake);

  private final TalonFXSimState  m_motorSim        = m_intakeRotary.getSimState( );
  private final CANcoderSimState m_CANCoderSim     = m_CANCoder.getSimState( );

  // Declare module variables
  private boolean                m_motorValid;      // Health indicator for Falcon 
  private boolean                m_ccValid;         // Health indicator for CANCoder 
  private boolean                m_calibrated      = true;
  private boolean                m_debug           = true;

  private INMode                 m_mode            = INMode.INTAKEROTARY_INIT;     // Manual movement mode with joysticks

  private static double          m_currentDegrees  = 0.0; // Current angle in degrees
  private double                 m_targetDegrees   = 0.0; // Target angle in degrees

  private boolean                m_moveIsFinished;        // Movement has completed (within tolerance)

  private VoltageOut             m_requestVolts    = new VoltageOut(0).withEnableFOC(false);
  private MotionMagicVoltage     m_requestMMVolts  = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private double                 m_totalArbFeedForward;   // Arbitrary feedforward added to counteract gravity

  private Timer                  m_safetyTimer     = new Timer( ); // Safety timer for movements
  private StatusSignal<Double>   m_motorPosition   = m_intakeRotary.getRotorPosition( );
  private StatusSignal<Double>   m_motorVelocity   = m_intakeRotary.getRotorVelocity( );
  private StatusSignal<Double>   m_motorCLoopError = m_intakeRotary.getClosedLoopError( );
  private StatusSignal<Double>   m_motorSupplyCur  = m_intakeRotary.getSupplyCurrent( );
  private StatusSignal<Double>   m_motorStatorCur  = m_intakeRotary.getStatorCurrent( );

  // Manual config parameters

  //Devices and simulation objs

  // Constructor
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    initialize( );

    m_motorValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_intakeRotary, "Intake Rotary", CTREConfigs6.intakeRotaryFXConfig( ));
    m_ccValid =
        PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANCoder, "Intake Rotary", CTREConfigs6.intakeRotaryCancoderConfig( ));

  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_motorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANCoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));

  }

  private void initSmartDashboard( )
  {}
  // Initialize dashboard widgets

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    setStopped( );

    //m_currentDegrees = getTalonFXDegrees( );
    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////

  public double getAngle( )
  {
    return m_currentDegrees;
  }

  public void setStopped( )
  {
    DataLogManager.log(String.format("%s: now STOPPED", getSubsystem( )));
    m_intakeRotary.setControl(m_requestVolts.withOutput(0.0));
  }

  ///////////////////////// MANUAL MOVEMENT ///////////////////////////////////

  public void moveWithJoystick(XboxController joystick)
  {
    double axisValue = -joystick.getRightY( );
    boolean rangeLimited = false;
    INMode newMode = INMode.INTAKEROTARY_STOPPED;

  }
}
