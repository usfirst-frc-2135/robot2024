package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;
import edu.wpi.first.math.MathUtil;


public class Robot extends TimedRobot
{
  private final static double                       kDt           = 0.020;  // 20 msec per RoboRIO loop

  private final static double                       kv            = 8.0;    // Max velocity - RPS
  private final static double                       ka            = 16.0;   // Max acceleration - RPS^2

  private final static double                       goal_1        = 0.0;    // Goal 1 position
  private final static double                       goal_2        = 0.5;    // Goal 2 position

  private final static XboxController               m_controller  = new XboxController(1);
  private final static ExampleSmartMotorController  m_motor1       = new ExampleSmartMotorController(5);
  private final static ExampleSmartMotorController  m_motor2       = new ExampleSmartMotorController(6);
  
  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  private TrapezoidProfile.State                    m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint    = new TrapezoidProfile.State( );

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
    // button pressed --> constant speed
    if (m_controller.getBButton())
    {
      motorOutput = 0.5;
    }
    else if (m_controller.getYButton( )) //custom
    {
      motorOutput = (SmartDashboard.getNumber("CustomSpeed", motorOutput));
    }  
    else if (m_controller.getAButton( ))
    {
      motorOutput = 1;
    }
    else if (m_controller.getLeftBumper()) //controller
    {
      double stickValue= m_controller.getRightY();
      double stickDeadband = MathUtil.applyDeadband(stickValue, 0.15);
      motorOutput = stickDeadband;
    }
    // else if (m_controller.getRightBumper()){
    //   m_goal = new TrapezoidProfile.State(goal_1, 0);
    //   TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    //   m_setpoint = profile.calculate(kDt);
    //   m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);

    // }
    // else if(m_controller.getYButton()){
    //   m_goal = new TrapezoidProfile.State(goal_2, 0);
    //   TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    //   m_setpoint = profile.calculate(kDt);
    //   m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
    // }
    else {
      //motorOutput = 0;
    }



    m_motor1.set(motorOutput); 
    m_motor2.set(-1*motorOutput); 
    SmartDashboard.putNumber("MotorVoltage", motorOutput);
    
  }

  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic();
    m_motor2.periodic();
  }

}