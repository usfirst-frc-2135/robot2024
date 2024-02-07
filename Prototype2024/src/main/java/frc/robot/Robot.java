package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot
{
  private final static XboxController              m_controller = new XboxController(1);
  private final static ExampleSmartMotorController m_motor1     = new ExampleSmartMotorController(5);
  private final static ExampleSmartMotorController m_motor2     = new ExampleSmartMotorController(6);

  @Override
  public void robotInit( )
  {
    double def = 0.5;
    SmartDashboard.putNumber("CustomSpeed", def);
    SmartDashboard.putNumber("MotorVoltage", 0);
  }

  @Override
  public void teleopPeriodic( )
  {
    double motorOutput = 0.0;
    if (m_controller.getBButton( ))
      motorOutput = 0.5;
    else if (m_controller.getYButton( ))
      motorOutput = (SmartDashboard.getNumber("CustomSpeed", motorOutput));
    else if (m_controller.getAButton( ))
      motorOutput = 1;
    else if (m_controller.getLeftBumper( ))
    {
      double stickValue = m_controller.getRightY( );
      double stickDeadband = MathUtil.applyDeadband(stickValue, 0.15);
      motorOutput = stickDeadband;
    }
    m_motor1.set(motorOutput);
    m_motor2.set(-1 * motorOutput);
    SmartDashboard.putNumber("MotorVoltage", motorOutput);
  }

  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic( );
    m_motor2.periodic( );
  }

}
