package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;


public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax neoIntake;
    private TalonSRX rightMotor;
    private TalonSRX leftMotor;
    public AnalogInput sharp = new AnalogInput(0);
    

    
    public IntakeSubsystem(){
        rightMotor = new TalonSRX(7);
        leftMotor = new TalonSRX(6);
        neoIntake = new CANSparkMax(36, MotorType.kBrushless);

        rightMotor.setInverted(true);
        leftMotor.setInverted(true);
        neoIntake.setInverted(true);
        
        rightMotor.setSelectedSensorPosition(0);
        leftMotor.setSelectedSensorPosition(0);

    }

    public void setIntake(Boolean open){
        if (open == true){
            rightMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_SPEED);
            leftMotor.set(ControlMode.PercentOutput,  Constants.Intake.INTAKE_SPEED);
            neoIntake.set(0);

        }
        else{
            rightMotor.set(ControlMode.PercentOutput, 0.0);
            leftMotor.set(ControlMode.PercentOutput, 0.0);
            neoIntake.set(0);
        }
    }
    public void setIntakeReversed(){
     
        rightMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_REVERSE_SPEED);
        leftMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_REVERSE_SPEED);
        neoIntake.set(-Constants.Intake.INTAKE_TO_SHOOTER_SPEED);

    }
    public void intakeToShooter(){
        neoIntake.set(Constants.Intake.INTAKE_TO_SHOOTER_SPEED);
        rightMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_SPEED);
        leftMotor.set(ControlMode.PercentOutput,  Constants.Intake.INTAKE_SPEED);
    }
    public void autoIntake(){
      
        rightMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_AUTO_SPEED);
        leftMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_AUTO_SPEED);
    }
    public void hizliIntake(){
        rightMotor.set(ControlMode.PercentOutput, 0.85);
        leftMotor.set(ControlMode.PercentOutput, 0.85);
    }

    @Override
    public void periodic(){
        double val = sharp.getValue();
        if (val > Constants.Intake.INTAKE_SHARP_VALUE_LIMIT){
            SmartDashboard.putBoolean("Intake Durum", true);
        }
        else {
            SmartDashboard.putBoolean("Intake Durum", false);
        }
    }
}