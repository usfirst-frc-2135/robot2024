package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  private final static double                       kDt           = 0.020;  // 20 msec per RoboRIO loop
  private final static double                       kv            = 8.0;    // Max velocity - RPS
  private final static double                       ka            = 16.0;   // Max acceleration - RPS^2
  private final static double                       goal_1        = 0.0;    // Goal 1 position
  private final static double                       goal_2        = 0.5;    // Goal 2 position
  private double                                    m_motorOutput = 0.0;
  private final static XboxController               m_controller  = new XboxController(1);
  private final static ExampleSmartMotorController  m_motor1      = new ExampleSmartMotorController(5);
  private boolean                                   m_closedLoop  = false;
  private final static ExampleSmartMotorController  m_motor2      = new ExampleSmartMotorController(6);
  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  private TrapezoidProfile.State                    m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint    = new TrapezoidProfile.State( );

  @Override
  public void robotInit( )
  {
    SmartDashboard.putNumber("CustomSpeed", 0.5);
    SmartDashboard.putNumber("MotorOutput", 0);
  }

  @Override
  public void teleopPeriodic( )
  {
    // if (m_controller.getLeftBumper( ))
    {
      if (m_controller.getAButton( )) //custom
        m_motorOutput = (SmartDashboard.getNumber("CustomSpeed", m_motorOutput));
      else if (m_controller.getBButton( )) //custom but reversed
        m_motorOutput = -(SmartDashboard.getNumber("CustomSpeed", m_motorOutput));
      else if (m_controller.getLeftBumper( )) //joystick
        m_motorOutput = MathUtil.applyDeadband(m_controller.getRightY( ), 0.15);
      else if (m_controller.getRightBumper( )) //reset
        m_motorOutput = 0;
      SmartDashboard.putNumber("MotorOutput", m_motorOutput);
      m_motor1.set(m_motorOutput);
      m_motor2.set(-1 * m_motorOutput);
    }

    if (m_controller.getXButton( )) //setpoint: 0
    {
      m_goal = new TrapezoidProfile.State(goal_1, 0);
      SmartDashboard.putNumber("Goal", m_goal.position);
    }
    if (m_controller.getYButton( )) //setpoint: 1
    {
      m_closedLoop = true;
      m_goal = new TrapezoidProfile.State(goal_2, 0);
      SmartDashboard.putNumber("Goal", m_goal.position);
    }

    // if (m_closedLoop = true)
    // {
    //     TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    //     m_setpoint = profile.calculate(kDt);
    //     m_motor1.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
    //     m_motor2.setSetpoint(PIDMode.kPosition, -m_setpoint.position, 0.0);
    //     SmartDashboard.putNumber("Setpoint", m_setpoint.position);
    // }
  }

  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic( );
    m_motor2.periodic( );
  }
}
