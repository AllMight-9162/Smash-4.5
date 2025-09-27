// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

/**
 * Classe de constantes
 */
public final class Constants {
  // Aqui temos várias constantes referentes as demais áreas do robô
    
  public static final class Dimensoes {
    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double ROBOT_MASS = 45;
    //Velocidade máxima *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double MAX_SPEED = 2.0;
    //Posição do módulo mais longe *COLOQUE OS MESMOS VALORES DO JSON*
    private static final Translation2d FURTHEST_MODULE_POSE = new Translation2d(MAX_SPEED, LOOP_TIME);
    public static final double MAX_ANGULAR_SPEED = SwerveMath.calculateMaxAngularVelocity(MAX_SPEED, FURTHEST_MODULE_POSE.getX(), FURTHEST_MODULE_POSE.getY());

    //Posições do centro de massa *DEVE SER CONFIGURADO PARA SEU ROBÔ*
    private static final double xMass = 0;
    private static final double yMass = 0;
    private static final double zMass = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    // Contem a porta em que o controle está
    public static final class Controle {
      // Porta do controle
      public static final int xboxControle = 0;
      
      // Deadband do controle
      public static final double DEADBAND = 0.1;
    }

    public static final class SwerveConfigs {
      // variável que ativa o PID para controlar a orientação do robô (PID tunado no json)
      public static final boolean headingCorrection = false;
      // true para correção de aceleração
      public static final boolean accelCorrection = false;
      // constante para diminuir o input do joystick (0 < multiplicadorRotacional <= 1)
      public static final double multiplicadorRotacional = 0.8;
      // constante para diminuir o input do joystick (y)
      public static final double multiplicadorTranslacionalY = 0.7;
      // constante para diminuir o input do joystick (x)
      public static final double multiplicadorTranslacionalX = 0.7;
      public static final double TURN_CONSTANT = 0.75;
      // variável que ativa a coração para resolver problema de "skew"
      public static final boolean usarCorrecaoDesvioVelocidadeAngular = false;
      // constante que corrije o skew (de -0.15 - 0.15) ESSA VARIÁVEL DEVE SER TUNADA PARA SEU ROBÔ
      public static final double coeficienteCorecaoAngVel = 4;
    }
}