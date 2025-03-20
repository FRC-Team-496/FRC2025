package frc.robot.subsystems;

public class SALUS {
    private double speed = 0.4;
    private boolean badX;
    private boolean badYaw;
    private boolean badY;

    public SALUS(){

    }

    public double calcX(){
        if(badX){
            double x = Camera.getX();
            if(x > 20){
                return -speed;
            }
            else if(x < -20){
                return speed;
            }
            else if( x < 20 && x > 0){
                return -x / 50.0;
            }
            else if( x > -20 && x < 0){
                return -x / 50.0;
            }
            else{
                badX = false;
                badY = false;
                return 0.0;
            }
        }
        return 0.0;
    }

    public double calcYaw(){
        if(badYaw){
            double yaw = Camera.getYaw();
            if(yaw > 0 && yaw < 160){
                return speed;
            }
            else if(yaw < 0 && yaw > -160){
                return -speed;
            }
            else if(yaw < -160 && yaw > -178){
                return -(yaw + 180.0) / 50.0;
            }
            else if(yaw > 160 && yaw < 178){
                return -(yaw - 180.0) / 50.0;
            }
            else{
                badYaw = false;
                badX = true;
                return 0.0;
            }
        }
        else{
            return 0.0;
        }
    }

    // public double calcY(){
        
    // }

    public void set(){
        badYaw = true;
        badX = false;
        badY = false;
    }
}