//
// Feeder Subystem - takes in Notes and feeds them into the Amp and Trap
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

//
// Feeder subsystem class
//
public class Feeder extends SubsystemBase
{
  // Member objects
  private final WPI_TalonSRX m_feederRoller = new WPI_TalonSRX(Ports.kCANID_FeederRoller);
  private final TalonFX      m_feederRotary = new TalonFX(Ports.kCANID_FeederRotary);
  private final CANcoder     m_CANCoder     = new CANcoder(Ports.kCANID_FeederCANCoder);
  private final DigitalInput m_noteInFeeder = new DigitalInput(Ports.kDIO1_NoteInFeeder);

  //Devices and simulation objs

  // Constructor
  public Feeder( )
  {
    setName("Feeder");
    setSubsystem("Feeder");
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

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }
}
