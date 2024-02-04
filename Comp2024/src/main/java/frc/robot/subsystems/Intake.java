//
// Intake Subystem - takes in Notes and delivers them to the Shooter
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.util.PhoenixUtil5;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{
  // Member objects
  private final WPI_TalonSRX m_intakeRoller = new WPI_TalonSRX(Ports.kCANID_IntakeRoller);
  private final TalonFX      m_intakeRotary = new TalonFX(Ports.kCANID_IntakeRotary);
  private final CANcoder     m_CANCoder     = new CANcoder(Ports.kCANID_IntakeCANCoder);
  private final DigitalInput m_notInIntake  = new DigitalInput(Ports.kDIO0_NoteInIntake);
  private boolean            m_intakeValid;
  //Devices and simulation objs

  // Constructor
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    m_intakeValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_intakeRoller, "Intake Roller");
    SmartDashboard.putBoolean("HL_validIN", m_intakeValid);
    // TODO needs to be initialized 

    intakeTalonInitialize(m_intakeRoller, INConsts.kInvertMotor);
    initialize( );
  }

  @Override
  public void periodic( )
  {
    double currentDraw = m_intakeRoller.getStatorCurrent( );
    SmartDashboard.putNumber("INRoller_currentDraw", currentDraw);
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {}

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
    m_intakeRoller.set(output);
  }

}
