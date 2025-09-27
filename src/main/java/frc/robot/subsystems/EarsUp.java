package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EarsUp {

    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMaxConfig motorConfig;

    RelativeEncoder leftEncoder;
    SparkClosedLoopController leftPIDcontroller;

    RelativeEncoder rightEncoder;
    SparkClosedLoopController rightPIDcontroller;

    private final int idleftMotor = 17;
    private final int idrightMotor = 18;

    public double kP, kI, kD ,kFF ,kIz , outMax, outMin;

    public EarsUp(){
        leftMotor = new SparkMax(idleftMotor, MotorType.kBrushless);
        rightMotor = new SparkMax(idrightMotor, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPosition(0.0);
        leftPIDcontroller = leftMotor.getClosedLoopController();

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0.0);
        rightPIDcontroller = rightMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();
        motorConfig.encoder.positionConversionFactor(1);
        motorConfig.encoder.velocityConversionFactor(1);

        config(leftMotor);
        config(rightMotor);

    }

    public void config(SparkMax motor){
        kP = 0.1;
        kI = 0.000001;
        kD = 0.5;
        kFF = 0.0;
        kIz = 0.0;

        outMax = 1.0;
        outMin = -1.0;

        motorConfig.closedLoop.pidf(kP, kI, kD, kFF);
        motorConfig.closedLoop.iZone(kIz);
        motorConfig.closedLoop.outputRange(outMin, outMax);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void angulate(double position){
        leftPIDcontroller.setReference(( -position ) , ControlType.kPosition);
        rightPIDcontroller.setReference(( position) , ControlType.kPosition);
        
    }
}