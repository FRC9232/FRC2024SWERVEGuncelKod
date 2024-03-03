package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeSetCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private final ColorSubsystem colorSubsystem;
    private String durum;
    private boolean finished;
    public IntakeSetCommand(IntakeSubsystem intakeSubsystem,ColorSubsystem colorSubsystem,String durum){
        this.intakeSubsystem = intakeSubsystem;
        this.colorSubsystem = colorSubsystem;
        this.durum = durum;

        addRequirements(intakeSubsystem, colorSubsystem);
    }
    public void execute(){
        finished = false;
        switch(durum){
            
            case "intake":
                intakeSubsystem.setIntake(colorSubsystem.a);
                finished = !colorSubsystem.a;
                break;
            case "intakeToShooter":
                intakeToShooter();
                finished=true;
                break;
            case "outtake":
                intakeSubsystem.setIntakeReversed();
                finished=true;
                break;
            case "stop":
                intakeSubsystem.setIntake(false);
                finished = true;
                break;
            case "autoIntake":
                autoIntake();
                finished = true;
                break;
        }
        
        isFinished();
        
        SmartDashboard.putBoolean("Intake Durum", colorSubsystem.a);
    }
    public void intakeClose(){
        intakeSubsystem.setIntake(false);
    }
    public void intakeToShooter(){
        intakeSubsystem.intakeToShooter();

    }
    public void autoIntake(){
        intakeSubsystem.autoIntake();

    }
    @Override
    public void end(boolean interrupted) {
        if(durum == "outtake" || durum == "intakeToShooter"){}
        else{
            intakeSubsystem.setIntake(false);
        }
        
  }
    @Override
    public boolean isFinished() {
        return finished;
  }
}
