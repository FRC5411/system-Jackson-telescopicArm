package frc.robot.teleArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.teleArm.ArmConstants.tArmConsts;


public class TeleSparkMAX extends SubsystemBase {
    private CANSparkMax rightShaft;   
    private CANSparkMax leftShaft;  

    private RelativeEncoder armEncoder; 
    
    private SparkPIDController pidControl; 
    

    public TeleSparkMAX(){ 
      // Shaft motors
        rightShaft = new CANSparkMax(ArmConstants.tArmConsts.rightShaftCANID, MotorType.kBrushless);  
        leftShaft = new CANSparkMax(ArmConstants.tArmConsts.leftShaftCANID, MotorType.kBrushless);  
        
        rightShaft.restoreFactoryDefaults(); 
        leftShaft.restoreFactoryDefaults(); 
        
        leftShaft.follow(rightShaft);   
        
      // Encoder
        armEncoder = rightShaft.getEncoder();  
        armEncoder.setPosition(0); 
      // PID controller
        pidControl = rightShaft.getPIDController();   

        pidControl.setP(ArmConstants.tArmConsts.armOneP); 
        pidControl.setI(ArmConstants.tArmConsts.armOneI); 
        pidControl.setD(ArmConstants.tArmConsts.armOneD); 

        pidControl.setFeedbackDevice(armEncoder); 

        rightShaft.burnFlash(); 
        leftShaft.burnFlash(); 


        
    } 

    public void extend(){ 
      pidControl.setReference(tArmConsts.armLengthMet - tArmConsts.spaceMarginMet,
      ControlType.kPosition);
    }  
    
    public void retract(){ 
      pidControl.setReference(tArmConsts.spaceMarginMet,ControlType.kPosition); 
    } 

    public void setVelocity(double velo){ 
      pidControl.setReference(velo, ControlType.kVelocity); 
    }
    

}