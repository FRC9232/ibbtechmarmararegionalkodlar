package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimbSubsystem;




public class ClimbSetCommand extends Command {
    
    private final ClimbSubsystem climbSubsystem;
    private int open;

    
    public ClimbSetCommand(ClimbSubsystem climbSubsystem, int open){
        this.climbSubsystem = climbSubsystem;
        this.open = open;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        switch(open){
            case 1:
                climbSubsystem.setClimb(1);
                break;
            case 0:
                climbSubsystem.setClimb(0);
                break;
            case -1:
                climbSubsystem.setClimb(-1);
                break;
                
        }
    }

    public void execute(){}

    @Override
    public boolean isFinished() {
        return true;
    }
    













 }