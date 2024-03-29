package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.*;

public class ClimbSubsystem extends SubsystemBase {
    public TalonSRX climbMotor;
    public DigitalInput limitSwitch = new DigitalInput(2);

    public ClimbSubsystem(){
        climbMotor = new TalonSRX(5);
        climbMotor.setSelectedSensorPosition(0);
        }

    public void setClimb(int open){
        if (open == 1){
            climbMotor.set(ControlMode.PercentOutput, -1 * Constants.Climb.CLIMB_SPEED);
        }
        else if(open == -1){
            climbMotor.set(ControlMode.PercentOutput, Constants.Climb.CLIMB_SPEED);
        }
        else{
            climbMotor.set(ControlMode.PercentOutput, 0);
        }

    }

    
}