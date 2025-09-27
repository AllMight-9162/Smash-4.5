package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  Timer timer;
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    timer = new Timer();
    
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.periodic();
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("TX da limelight", m_robotContainer.getTX);
    SmartDashboard.putNumber("TA da limelight", m_robotContainer.getTA);
    SmartDashboard.putNumber("coralPosition", m_robotContainer.coralPosition);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    m_robotContainer.autoInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleOpinit();
  }
  
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleOP();
  }

}