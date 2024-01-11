package frc.robot.teleArm;

//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.teleArm.ArmConstants.tArmConsts; 
import frc.robot.teleArm.ArmConstants.tArmConsts;

public class TeleTalonFX extends SubsystemBase {
    private TalonFX leftShaft; 
    private TalonFX rightShaft;   

    private PIDController pidControl; 
    public TeleTalonFX(){   

        leftShaft = new TalonFX(tArmConsts.leftShaftTalonID);
        rightShaft = new TalonFX(tArmConsts.rightShaftTalonID); 
        
        rightShaft.setControl(new Follower(leftShaft.getDeviceID(),false));
        
        var shaftConfig = new TalonFXConfiguration(); 
        shaftConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        shaftConfig.CurrentLimits.StatorCurrentLimit = 30;  

        leftShaft.getConfigurator().apply(shaftConfig); 
        rightShaft.getConfigurator().apply(shaftConfig);    
        
        
        pidControl = new PIDController(tArmConsts.armOneP, tArmConsts.armOneI, tArmConsts.armOneD); 
        pidControl.setTolerance(3); 
        
    }   


    public void extend(){ 
        double output = pidControl.calculate(rightShaft.getPosition().getValueAsDouble(),  
        tArmConsts.armLengthMet - tArmConsts.spaceMarginMet); 

        rightShaft.set(output); 
        leftShaft.set(output); 
    } 

    public void retract() { 
        double output = pidControl.calculate(tArmConsts.spaceMarginMet,  
        tArmConsts.armLengthMet - tArmConsts.spaceMarginMet); 

        rightShaft.set(output); 
        leftShaft.set(output); 
    } 

    public void setVelocity(double velocity){ 
        rightShaft.setControl(new VelocityVoltage(velocity)); 
        leftShaft.setControl(new VelocityVoltage(velocity));
    } 
}
