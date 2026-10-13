package frc.team5115.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;

import frc.team5115.util.MotorContainer;
import java.util.ArrayList;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements MotorContainer {
    public IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    

    public Intake(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    
    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }

    public Command run(double speed){
        return Commands.run(() -> io.setPercent(speed), this);
    }

    public Command stop(){
        return run(0);
    }

    public Command intake(){
        return run(Constants.INTAKE_SPEED);
    }

    public Command vomit(){
        return run(Constants.INTAKE_VOMIT_SPEED);
    }
    

    @AutoLogOutput
    public boolean isIntaking() {
        return inputs.velocityRPM > 50;
    }

    @AutoLogOutput
    public boolean isVomiting() {
        return inputs.velocityRPM < 50;
    }

    

}
