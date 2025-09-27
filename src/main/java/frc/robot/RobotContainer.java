package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.commands.AutoAlignDriveCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.KelpIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.EarsUp;
import frc.robot.subsystems.Utilities.Limelight;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Inicializa subsistemas
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final SendableChooser<Command> autoChooser;
  private CommandXboxController controleXbox = new CommandXboxController(Controle.xboxControle);

  CoralIntake coralIntake = new CoralIntake();
  KelpIntake kelpIntake = new KelpIntake();
  Climb climb = new Climb();
  EarsUp earsUp = new EarsUp();
  Limelight limelights = new Limelight();
  XboxController pilotoSub = new XboxController(1);

  public boolean n1,n2,n3;
  public boolean kelpColect,processor,station,stop, climbUp;
  public boolean kelpCntrl, climbCntrl = true;
  public double coralPosition, kelpPosition;
  public double getTV, getTA, getTX;
  public double TargetArea;

  public RobotContainer() {
    // Define o comando padrão como controle manual da Swerve
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
      () -> controleXbox.getRightX()));

    NamedCommands.registerCommand("n3", Commands.runOnce(() -> coralIntake.angulate(9.1)));
    NamedCommands.registerCommand("n2", Commands.runOnce(() -> coralIntake.angulate(1.3)));
    NamedCommands.registerCommand("score", Commands.runOnce(() -> coralIntake.drop(0.8)));
    NamedCommands.registerCommand("reset", Commands.runOnce(() -> coralIntake.angulate(1.3)));
    NamedCommands.registerCommand("stop", Commands.runOnce(() -> coralIntake.stop()));
    NamedCommands.registerCommand("stopRobot", Commands.runOnce(() -> swerve.stopRobot()));
    NamedCommands.registerCommand("align", Commands.runOnce(() -> new AutoAlignDriveCommand(swerve, limelights, controleXbox)));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }
  private void configureBindings() {
    if(!Robot.isReal()){
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));}
  }

  public void periodic() 
  {
    controleXbox.leftBumper().whileTrue(new AutoAlignDriveCommand(swerve, limelights, controleXbox));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void autoInit() {
    earsUp.angulate(2.0);
    setMotorBrake(true);
  }

  public void teleOpinit() {
    setFalse();
    earsUp.angulate(2.0); 
  }

  public void teleOP() {
    if(pilotoSub.getAButtonPressed()){
      setFalse();
      stop = false;
      n1 = true;
    }else if(pilotoSub.getBButtonPressed()){
      setFalse();
      stop = false;
      n2 = true;
    }else if(pilotoSub.getYButtonPressed()){
      setFalse();
      stop = false;
      n3 = true;
    }else if(pilotoSub.getXButtonPressed()){
      setFalse();
      stop = false;
      climbUp = true;
    }else if(pilotoSub.getRightBumperButtonPressed()){
      if(n1 == true || n2 == true || n3 == true || processor == true || climbUp == true){
        processor = false;
      }
      else{
        setFalse();
        stop = false;
        processor = true;
      }
    }else if(pilotoSub.getLeftBumperButtonPressed()){
      if(n1 == true || n2 == true || n3 == true || processor == true || climbUp == true){
        station = false;
      }else{
        setFalse();
        stop = false;
        station = true;
      }
    }else if(pilotoSub.getStartButtonPressed()){
      setFalse();
      stop = false;
      kelpColect = true;
    }else if(pilotoSub.getBackButtonPressed()){
      setFalse();
      stop = true;
    }

    n1();
    n2();
    n3();
    station();
    processor();
    stop();
    getKelp();
    climb();
  }

  public void n1() {
    if (n1) {
      getTA = limelights.getBackTA();
      getTX = limelights.getBackTX();
      getTV = limelights.getBackTV();
      coralIntake.angulate(0.8);
      if (pilotoSub.getLeftBumperButton()) {
        coralIntake.drop(0.20);
        earsUp.angulate(0.00);
      }
    }
  }

  public void n2() {
    if (n2) {
      getTA = limelights.getBackTA();
      getTX = limelights.getBackTX();
      getTV = limelights.getBackTV();
      coralIntake.angulate(1.3);
      if (pilotoSub.getLeftBumperButton()) {
        coralIntake.drop(0.85);
        earsUp.angulate(0.00);
      }
    }
  }

  public void n3() {
    if (n3) {
      getTA = limelights.getFrontTA();
      getTX = limelights.getFrontTX();
      getTV = limelights.getFrontTV();
      coralIntake.angulate(9.1);
      if (pilotoSub.getLeftBumperButton()) {
        coralIntake.drop(0.8);
        earsUp.angulate(0.00);
      }
    }
  }

  public void station() {
    if (station) {
      coralIntake.angulate(3.0);
      coralIntake.take();
      earsUp.angulate(2.00);
    }
  }

  public void processor() {
    if (processor) {
      kelpIntake.angulate(4.0);
      kelpIntake.drop();
      kelpCntrl = false;
      earsUp.angulate(0.00);
    }
  }

  public void getKelp() {
    if (kelpColect) {
      kelpIntake.angulate(6.5);
      kelpIntake.take();
      kelpCntrl = true;
      earsUp.angulate(2.0);
    }
  }

  public void climb() {
    if (climbUp) {
      climbCntrl = false;
      coralIntake.angulate(9.5);
      if (pilotoSub.getRightBumperButton()) {
        climb.climbUp(0.25);
      } else if (pilotoSub.getLeftBumperButton()) {
        climb.climbUp(-0.1);
      }
    }
  }

  public void stop() {
    if (stop) {
      setFalse();
      coralIntake.angulate(1.7);
      coralIntake.stop();
      kelpIntake.stop();
      if (kelpCntrl) {
        kelpIntake.angulate(4.0);
      } else {
        kelpIntake.reset();
      }
    }
  }

  public void setFalse() {
    n1 = false;
    n2 = false;
    n3 = false;

    kelpColect = false;
    processor = false;
    station = false;
    climbUp = false;
  }
}
