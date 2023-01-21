package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class CommandExtendClimber extends InstantCommand {
    public CommandExtendClimber(ClimberSubsystem climberSubsystem) {
        super(() -> {
            climberSubsystem.setCylinders(true);
            SmartDashboard.putBoolean("IsRunning", true);
        }, climberSubsystem);
    }
}
