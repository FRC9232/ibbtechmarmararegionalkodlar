package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeSetCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private String durum;
    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, String durum){
        this.intakeSubsystem = intakeSubsystem;
        this.durum = durum;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize(){
        switch(durum){
            
            case "intake":
                
            System.out.println("intake command true");
                intakeSubsystem.setIntake(true);
                break;
            
            case "intakeToShooter":
                
                intakeSubsystem.intakeToShooter();
                break;
            
            case "outtake":
                
                intakeSubsystem.setIntakeReversed();
                break;
            
            case "stop":
                
                intakeSubsystem.setIntake(false);
                break;
            
            case "autoIntake":
                
                intakeSubsystem.autoIntake();
                break;
            case "hizliIntake":
                
                intakeSubsystem.hizliIntake();
                break;
        }
    }
    public void execute(){}
    
    @Override
    public boolean isFinished() {
        if(durum == "intake"){
            double val = intakeSubsystem.sharp.getValue();
            if (val > Constants.Intake.INTAKE_SHARP_VALUE_LIMIT){
                System.out.println("Sharp is ok");
                intakeSubsystem.setIntake(false);
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return true;
        }
  }
}
