package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Utilities.Limelight;
import frc.robot.Constants.Controle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoAlignDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final Limelight limelight;
  private final CommandXboxController controle;

  public AutoAlignDriveCommand(SwerveSubsystem swerve, Limelight limelight, CommandXboxController controle) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.controle = controle;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double tx = limelight.getFrontTX(); // Deslocamento lateral da tag
    double ta = limelight.getFrontTA(); // Área da tag
    boolean temTag = limelight.getFrontTV() == 1.0;

    // Movimento manual com controle
    double vx = MathUtil.applyDeadband(controle.getLeftY(), Controle.DEADBAND);
    double vy = MathUtil.applyDeadband(controle.getLeftX(), Controle.DEADBAND);
    double omega = MathUtil.applyDeadband(controle.getRightX(), Controle.DEADBAND);

    // Se houver tag visível e estiver perto, aplicar correção lateral automática
    if (temTag && ta > 3.0) {
      double kP = 0.06;
      double offset = 2.0;
      double erro = tx - offset;
      vy = MathUtil.clamp(kP * erro, -0.5, 0.5); // Substitui apenas o vy (lateral)
      vx = -0.4;
    }

    swerve.drive(new Translation2d(vx, vy), omega, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
