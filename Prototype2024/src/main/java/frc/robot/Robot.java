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

  private final static XboxController               m_controller  = new XboxController(1);
  private final static ExampleSmartMotorController  m_motor1      = new ExampleSmartMotorController(5);
  private final static ExampleSmartMotorController  m_motor2      = new ExampleSmartMotorController(6);

  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  private TrapezoidProfile.State                    m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint    = new TrapezoidProfile.State( );

  @Override
  public void robotInit( )
  {
    double def = 0.5;
    SmartDashboard.putNumber("CustomSpeed", def);
    SmartDashboard.putNumber("MotorVoltage", 0);
    SmartDashboard.putString("tester", "nope");
  }

  @Override
  public void teleopPeriodic( )
  {

    //custom speed + joystick
    double motorOutput = 0.0;
    if ((m_controller.getAButtonPressed( )))
    {
      motorOutput = (SmartDashboard.getNumber("CustomSpeed", motorOutput));
      m_motor1.set(motorOutput);
      m_motor2.set(-1 * motorOutput);
      SmartDashboard.putNumber("MotorVoltage", motorOutput);
    }
    if ((m_controller.getBButtonPressed( )))
    {
      motorOutput = (SmartDashboard.getNumber("CustomSpeed", motorOutput));
      m_motor1.set(-1 * motorOutput);
      m_motor2.set(motorOutput);
      SmartDashboard.putNumber("MotorVoltage", motorOutput * -1);
    }
    if ((m_controller.getRightBumper( )))
    {
      motorOutput = (0);
      m_motor1.set(motorOutput);
      m_motor2.set(motorOutput);
      SmartDashboard.putNumber("MotorVoltage", motorOutput);
    }
    if (m_controller.getLeftBumper( ))
    {
      double stickValue = m_controller.getRightY( );
      double stickDeadband = MathUtil.applyDeadband(stickValue, 0.15);
      motorOutput = stickDeadband;
      m_motor1.set(motorOutput);
      m_motor2.set(-1 * motorOutput);
      SmartDashboard.putNumber("MotorVoltage", motorOutput);
    }

    //setpoint code 
    if (m_controller.getXButton( ))
    {
      m_goal = new TrapezoidProfile.State(goal_1, 0);
      DataLogManager.log("goal: " + m_goal);
      SmartDashboard.putNumber("0-goal", m_goal.position);
      DataLogManager.log("goal position: " + m_goal.position);

      TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal);
      m_setpoint = profile.calculate(kDt);

      m_motor1.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
      SmartDashboard.putNumber("1-setpoint", m_setpoint.position);

    }
    else if (m_controller.getYButton( ))
    {
      m_goal = new TrapezoidProfile.State(goal_2, 0);
      DataLogManager.log("goal: " + m_goal);
      SmartDashboard.putNumber("0-goal", m_goal.position);
      DataLogManager.log("goal position: " + m_goal.position);

      TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal);
      m_setpoint = profile.calculate(kDt);

      m_motor1.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
      SmartDashboard.putNumber("1-setpoint", m_setpoint.position);

    }

  }

  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic( );
    m_motor2.periodic( );
  }

}
