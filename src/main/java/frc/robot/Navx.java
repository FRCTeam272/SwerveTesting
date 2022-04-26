package frc.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Navx{
    public  AHRS navX; 
    public  double zeroHeading;
    public  double zeroAngle;

    
     // static variable single_instance of type Singleton 
     private static Navx single_instance = null; 
  
     // variable of type String 
     public String s; 
   
     // private constructor restricted to this class itself 
     private Navx() 
     { 
        navX = new AHRS(SPI.Port.kMXP);
        zeroHeading = navX.getFusedHeading();
        zeroAngle = navX.getAngle();
     } 
   
     // static method to create instance of Singleton class 
     public static Navx getInstance() 
     { 
         if (single_instance == null){ 
             single_instance = new Navx(); 
         }
         return single_instance; 
     }

     public void getFuzedHeading() {
        this.zeroHeading = this.navX.getFusedHeading();
     }
     public void getAngle() {
         this.zeroAngle = this.navX.getAngle();
     }
}