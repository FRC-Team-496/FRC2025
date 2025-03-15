package frc.robot.subsystems;

public class SALUS {
    private double speed = 0.4;
    

    public SALUS(){

    }

    public double calcX(){
        
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
           
        
        return 0.0;
    }

    public double calcYaw(){
        
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
                return 0.0;
            }
        
    }


    
    public double calcY(){
        
        double y = Camera.getY();
        if(y > 0 && y < 160){
            return speed;
        }
        else if(y < 0 && y > -160){
            return -speed;
        }
        else if(y < -160 && y > -178){
            return -(y + 180.0) / 50.0;
        }
        else if(y > 160 && y < 178){
            return -(y - 180.0) / 50.0;
        }
        else{
            return 0.0;
        }
    
}

    
}
