package frc.robot.commands;

import java.util.ArrayList;

import org.carlmontrobotics.lib199.swerve.SwerveModule.ModuleType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.TestType;
import frc.robot.subsystems.Drivetrain;

public class LogData extends CommandBase {

    private final ArrayList<double[]> data = new ArrayList<>();
    private final Drivetrain drivetrain;
    private double startTime;

    public LogData(Drivetrain drivetrain) {
        addRequirements(this.drivetrain = drivetrain);
    }

    @Override
    public void initialize() {
        data.clear();
        // drivetrain.resetEncoders();
        drivetrain.resetTurnLock();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        boolean drive = Robot.getDrive();
        if(drive && drivetrain.getTurnLock()) startTime = Timer.getFPGATimestamp();
        ModuleType module = Robot.getModuleType();
        data.add(new double[] {Timer.getFPGATimestamp(), drivetrain.getMotorVoltage(module, drive), drivetrain.getEncoderPosition(module, drive), drivetrain.getEncoderVelocity(module, drive)});
        double voltage = Robot.getVoltageCommand();
        if(Robot.getTestType() == TestType.QUASISTATIC) voltage *= Timer.getFPGATimestamp() - startTime;
        if(Robot.getDrive()) {
            drivetrain.driveRobot(voltage, Robot.getDirection());
        } else {
            drivetrain.driveTurnMotor(module, voltage);
            SmartDashboard.putNumber("encoderalsjf.ajsdf", drivetrain.turnEncoders[module.ordinal()].getPosition());
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("SysIdAckNumber", SmartDashboard.getNumber("SysIdAckNumber", 0) + 1);
        Robot.setTelemetry(data);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
