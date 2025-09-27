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

public class CoralIntake {
    
    SparkMax AngulateMotor;
    SparkMax LeftMotor;
    SparkMax RightMotor;

    RelativeEncoder encoder;
    SparkClosedLoopController PIDcontroller;
    SparkMaxConfig motorConfig;

    private final int IdAngulateMotor = 12;
    private final int IdLeftMotor = 13;
    private final int IdRightMotor = 14;
    
    public double kP, kI, kD ,kFF ,kIz , outMax, outMin;

    public CoralIntake(){

        AngulateMotor = new SparkMax(IdAngulateMotor, MotorType.kBrushless);
        LeftMotor = new SparkMax(IdLeftMotor, MotorType.kBrushless);
        RightMotor = new SparkMax(IdRightMotor, MotorType.kBrushless);

        encoder = AngulateMotor.getEncoder();
        encoder.setPosition(0.0);

        PIDcontroller = AngulateMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();
        motorConfig.encoder.positionConversionFactor(1);
        motorConfig.encoder.velocityConversionFactor(1);
        motorConfig.idleMode(IdleMode.kBrake);

        Config(AngulateMotor);
        Config(RightMotor);
        Config(LeftMotor);

    }

    public void Config(SparkMax motor){
        kP = 0.5;
        kI = 0.000001;
        kD = 2.0;
        kFF = 0.0;
        kIz = 0.0;

        outMax = 0.30;
        outMin = -0.30;

        motorConfig.closedLoop.pidf(kP, kI, kD, kFF);
        motorConfig.closedLoop.iZone(kIz);
        motorConfig.closedLoop.outputRange(outMin, outMax);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void angulate(double position){
        PIDcontroller.setReference(position, ControlType.kPosition);

    }

    public void take(){
        //LeftMotor.set(1.0);
        RightMotor.set(-1.0);

    }

    public void drop(double set){
        //LeftMotor.set(-position);
        RightMotor.set(set);

    }

    public void cima(double set){
        AngulateMotor.set(set);
    }

    public void baixo(double set){
        AngulateMotor.set(set);
    }

    public void reset(){
        angulate(0.0);
    }

    public void stop(){
        //LeftMotor.set(0.0);
        RightMotor.set(0.0);
        
    }

    public double getPosition(){
        return ((encoder.getPosition()));
    }

}