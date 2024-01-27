package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  private final static XboxController               m_controller    = new XboxController(1);
  private final static ExampleSmartMotorController  m_motor1        = new ExampleSmartMotorController(5);
  private final static ExampleSmartMotorController  m_motor2        = new ExampleSmartMotorController(6);

  private final static double                       kv              = 1.0;    // Max velocity - RPS
  private final static double                       ka              = 2.0;    // Max acceleration - RPS^2

  private final static TrapezoidProfile.Constraints m_constraints   = new TrapezoidProfile.Constraints(kv, ka);

  private final static double                       m_goal1         = 0.5;    // Goal 1 position
  private final static double                       m_goal2         = -1.0;   // Goal 2 position

  private boolean                                   m_closedLoop    = false;
  private double                                    m_percentOutput = 0.5;

  private Timer                                     m_timer         = new Timer( );
  private TrapezoidProfile                          m_profile       = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State                    m_goal          = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint      = new TrapezoidProfile.State( );

  @Override
  public void robotInit( )
  {
    SmartDashboard.putNumber("PercentOutput", m_percentOutput);
  }

  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic( );
    m_motor2.periodic( );
  }

  @Override
  public void disabledInit( )
  {
    m_closedLoop = false;
    m_goal.position = 0.0;
    m_goal.velocity = 0.0;
    m_setpoint.position = 0.0;
    m_setpoint.velocity = 0.0;
    m_motor1.set(0.0);
    m_motor2.set(0.0);
    m_motor1.resetEncoder( );
    m_motor2.resetEncoder( );
  }

  @Override
  public void teleopPeriodic( )
  {
    if (m_controller.getAButton( ) || m_controller.getBButton( ))
    {
      m_percentOutput = SmartDashboard.getNumber("PercentOutput", m_percentOutput);
      m_closedLoop = false;

      DataLogManager.log((m_controller.getAButton( ) ? "A" : "B") + " button pressed");
      double percentOutput = m_controller.getAButton( ) ? m_percentOutput : -m_percentOutput;
      m_motor1.set(percentOutput);
      m_motor2.set(-percentOutput);
    }
    else if (m_controller.getRightBumper( ))
    {
      DataLogManager.log("Right bumper pressed");
      m_closedLoop = false;
      m_motor1.set(0.0);
      m_motor2.set(0.0);
    }
    else if (m_controller.getXButton( ) || m_controller.getYButton( ))
    {
      DataLogManager.log((m_controller.getXButton( ) ? "X" : "Y") + " button pressed - new closed loop goal");
      if (!m_closedLoop)
      {
        m_closedLoop = true;
        m_timer.restart( );
        m_motor1.resetEncoder( );
        m_motor2.resetEncoder( );
        m_setpoint = new TrapezoidProfile.State( );
        m_goal = new TrapezoidProfile.State((m_controller.getXButton( ) ? m_goal1 : m_goal2), 0);
        DataLogManager.log("Start: goal: " + m_goal.position + " setpoint: " + m_setpoint.position + " Position: "
            + m_motor1.getEncoderPosition( ));

      }
    }
    else if (m_controller.getLeftBumper( ))
    {
      DataLogManager.log("Left bumper pressed");
      double stickValue = m_controller.getRightY( );
      m_percentOutput = MathUtil.applyDeadband(stickValue, 0.15);

      m_closedLoop = false;
      m_motor1.set(m_percentOutput);
      m_motor2.set(-m_percentOutput);
    }
    else if (m_closedLoop)
    {
      DataLogManager.log("Loop:  goal: " + m_goal.position + " setpoint: " + m_setpoint.position + " Position: "
          + m_motor1.getEncoderPosition( ));
      if (m_timer.hasElapsed(4.0) || Math.abs(m_motor1.getEncoderPosition( ) - m_goal.position) < 0.05)
      {
        DataLogManager.log("Return to open loop - " + (m_timer.hasElapsed(4.0) ? "timed out" : "at goal"));
        m_closedLoop = false;
        m_motor1.set(0.0);
      }
      else
      {
        DataLogManager.log("Loop:  goal: " + m_goal.position + " setpoint: " + m_setpoint.position + " Position: "
            + m_motor1.getEncoderPosition( ));
        m_setpoint = m_profile.calculate(m_timer.get( ), m_setpoint, m_goal);
        m_motor1.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
        m_motor2.set(0.0);
      }
    }

    SmartDashboard.putNumber("0-goal", m_goal.position);
    SmartDashboard.putNumber("m_motor1", m_motor1.get( ));
    SmartDashboard.putNumber("m_motor2", m_motor2.get( ));
  }

}
