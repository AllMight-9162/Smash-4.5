package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

public class KelpIntake {

    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax angulateMotor;

    RelativeEncoder encoder;
    SparkClosedLoopController PIDcontroller;
    SparkMaxConfig motorConfig;

    private final int IdAngulateMotor = 9;
    private final int IdLeftMotor = 11;
    private final int IdRightMotor = 10;

    public double kP, kI, kD ,kFF ,kIz , outMax, outMin;

    public KelpIntake(){

        leftMotor = new SparkMax(IdLeftMotor, MotorType.kBrushless);
        rightMotor = new SparkMax(IdRightMotor, MotorType.kBrushless);
        angulateMotor = new SparkMax(IdAngulateMotor, MotorType.kBrushless);

        encoder = angulateMotor.getEncoder();
        encoder.setPosition(0.0);

        
        PIDcontroller = angulateMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();
        motorConfig.encoder.positionConversionFactor(1);
        motorConfig.encoder.velocityConversionFactor(1);

        config(angulateMotor);
        config(leftMotor);
        config(rightMotor);
        
    }

    public void config(SparkMax motor){
        kP = 0.1;
        kI = 0.0000006;
        kD = 1.00;
        kFF = 0.0;
        kIz = 0.0;

        outMax = 0.5;
        outMin = -0.5;

        motorConfig.closedLoop.pidf(kP, kI, kD, kFF);
        motorConfig.closedLoop.iZone(kIz);
        motorConfig.closedLoop.outputRange(outMin, outMax);
        motorConfig.idleMode(IdleMode.kBrake);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }

    public void angulate(double position){
        PIDcontroller.setReference(position, ControlType.kPosition);
    }

    public void take(){
        leftMotor.set(-1.0);
        rightMotor.set(1.0);
    } 

    public void drop(){
        leftMotor.set(1.0);
        rightMotor.set(-1.0);
    }

    public boolean inPosition(double position){
        double currentPosition = getPosition();

        return(currentPosition >= position -0.2 && currentPosition <= position + 0.2);

    }

    public void stop(){
        leftMotor.set(0.0);
        rightMotor.set(0.0);

    }

    public void reset(){
        angulate(0.0);
    }

    public double getPosition(){
        return encoder.getPosition();
        
    }

}
