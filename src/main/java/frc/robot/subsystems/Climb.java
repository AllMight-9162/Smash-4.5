package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Climb {
    SparkFlex leftMotor;
    SparkFlex rightMotor;
    SparkFlexConfig motorConfig;
    
    RelativeEncoder leftencoder;
    SparkClosedLoopController leftPIDController;

    RelativeEncoder rightEncoder;
    SparkClosedLoopController rightPIDController;

    private final int idleftMotor = 15;
    private final int idrightMotor = 16;

    public double kP, kI, kD ,kFF ,kIz , outMax, outMin;

    public Climb(){
        leftMotor = new SparkFlex(idleftMotor, MotorType.kBrushless);
        rightMotor = new SparkFlex(idrightMotor, MotorType.kBrushless);

        leftencoder = leftMotor.getEncoder();
        leftencoder.setPosition(0.0);
        leftPIDController = leftMotor.getClosedLoopController();

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0.0);
        rightPIDController = rightMotor.getClosedLoopController();

        motorConfig = new SparkFlexConfig();
        motorConfig.encoder.positionConversionFactor(1);
        motorConfig.encoder.velocityConversionFactor(1);

        config(leftMotor);
        config(rightMotor);
    }

    public void config(SparkFlex motor){
        kP = 0.3;
        kI = 0.0000006;
        kD = 0.015;
        kFF = 0.0;
        kIz = 0.0;

        outMax = 1.0;
        outMin = -1.0;

        motorConfig.closedLoop.pidf(kP, kI, kD, kFF);
        motorConfig.closedLoop.iZone(kIz);
        motorConfig.closedLoop.outputRange(outMin, outMax);
        motorConfig.idleMode(IdleMode.kBrake);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void climbUp(double position){
        leftPIDController.setReference((position * 60) , ControlType.kPosition);
        rightPIDController.setReference(( -position * 60) , ControlType.kPosition);

    }

    public void set(double speed){
        leftMotor.set(-speed);
        rightMotor.set(speed);
    }
    

}
