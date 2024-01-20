//
// Intake Subystem - takes in Notes and delivers them to the Shooter
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//
// Intake subsystem class
//
public class Intake extends SubsystemBase
{
  // Member objects
  private final WPI_TalonSRX m_intakeRoller       = new WPI_TalonSRX(0);
  private final TalonFX      m_intakeRotor  = new TalonFX(0);
  private final CANcoder     m_CANCoder     = new CANcoder(0);
  private final DigitalInput m_limitSwitch1 = new DigitalInput(0);
  private final DigitalInput m_limitSwitch2 = new DigitalInput(0);

  //Devices and simulation objs

  // Constructor
  public Intake( )
  {
    setName("Intake");
    setSubsystem("Intake");
    initialize( );
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
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {}
}
