package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;



public class ShooterSetCommand extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private String durum;
    private boolean finished;
    public ShooterSetCommand(ShooterSubsystem shooterSubsystem,String durum){
        this.shooterSubsystem = shooterSubsystem;
        this.durum = durum;
        addRequirements(shooterSubsystem);
    } 
    public void execute(){
        finished = false;
        switch(durum){
            case "speaker":
                shooterSubsystem.setShooter(true);
                finished = true;
                break;
            case "amph":
                shooterSubsystem.amphShoot();
                finished = true;
                break;
            case "stop":
                shooterSubsystem.setShooter(false);
                finished = true;
                break;
        }
        isFinished();
    }
    
    
    
    public void StartShooter(){

        shooterSubsystem.setShooter(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        //shooterSubsystem.setShooter(false);
        
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}
