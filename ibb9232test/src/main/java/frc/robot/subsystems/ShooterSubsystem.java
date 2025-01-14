package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private SparkPIDController PIDController;
    private  RelativeEncoder m_encoder;

    
    public ShooterSubsystem(){
        
        rightMotor = new CANSparkMax(9,MotorType.kBrushless);
        leftMotor = new CANSparkMax(8, MotorType.kBrushless);
        PIDController = rightMotor.getPIDController();
        m_encoder = rightMotor.getEncoder();
        rightMotor.setInverted(true);
        leftMotor.follow(rightMotor, true);
        
    }

    public void setShooter(Boolean open){
        if (open == true){
            rightMotor.set(Constants.Shooter.SHOOTER_SPEAKER);
            //PIDController.setReference(Constants.Shooter.SHOOTER_VEL, CANSparkMax.ControlType.kVelocity);
        }
        else{
            rightMotor.set(0);
        }
    }
    public void amphShoot(){
         rightMotor.set(Constants.Shooter.SHOOTER_AMPH);
     
    }

    
    

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());
    }

}