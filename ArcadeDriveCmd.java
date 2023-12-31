// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ArcadeDriveCmd extends CommandBase {

    private final Drive drive;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(Drive drive, Supplier<Double> speedFunction, Supplier<Double> turnFunction)
    {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get() ;
        double realTimeTurn = turnFunction.get() ;
        
       
        drive.arcadeDrive(realTimeSpeed, -realTimeTurn);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}