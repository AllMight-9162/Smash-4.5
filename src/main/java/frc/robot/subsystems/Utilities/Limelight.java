package frc.robot.subsystems.Utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable backLimelight;
    NetworkTable frontLimelight;

    public Limelight(){

    backLimelight = NetworkTableInstance.getDefault().getTable("limelight-back");
    frontLimelight = NetworkTableInstance.getDefault().getTable("limelight-front");
    

    }

    public double getBackTV(){
        return backLimelight.getEntry("tv").getDouble(0.0);

    }
    public double getFrontTV(){
        return frontLimelight.getEntry("tv").getDouble(0.0);

    }

    public double getBackTA(){
        return backLimelight.getEntry("ta").getDouble(0.0);
    }

    public double getFrontTA(){
        return frontLimelight.getEntry("ta").getDouble(0.0);
    }

    public double getBackTX(){
        return backLimelight.getEntry("tx").getDouble(0.0);
    }

    public double getFrontTX(){
        return frontLimelight.getEntry("tx").getDouble(0.0);
    }

    public int getBackID(){
        return (int) backLimelight.getEntry("tid").getDouble(-1);
    }

    public int getFrontID(){
        return (int) frontLimelight.getEntry("tid").getDouble(-1);
    }

    public boolean highKelp(){

        if(getBackID() == 8){
            return true;

        }else{
            return false;
        }
    }

    public boolean isAligned(Double ta, double tx, double getTA, double getTX) {
        double gettx = getTX;
        double getta = getTA;
        
        return (gettx >= tx - 2.0 && gettx <= tx + 2.0) && (getta >= -ta - 1.0 && getta <= ta - 1.0);
    }
    
}
