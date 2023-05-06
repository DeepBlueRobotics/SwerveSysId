package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignChargingStation extends CommandBase {

    private final Drivetrain drivetrain;
    private double lastTime = -1;
    private boolean fwd, fieldOriented;

    public AlignChargingStation(Drivetrain drivetrain) {
        addRequirements(this.drivetrain = drivetrain);
    }

    @Override
    public void initialize() {
        fwd = Math.abs(getRoll()) > Math.abs(getPitch());
        fieldOriented = drivetrain.getFieldOriented();
        drivetrain.setFieldOriented(false);
    }

    @Override
    public void execute() {
        double roll = -getRoll();
        double pitch = getPitch();
        double forward = fwd && Math.abs(roll) > chargeStationAlignToleranceDeg ? chargeStationAlignSpeedMpSPerDeg * roll + Math.copySign(chargeStationAlignFFMpS, roll) : 0;
        double strafe = !fwd && Math.abs(pitch) > chargeStationAlignToleranceDeg ? chargeStationAlignSpeedMpSPerDeg * pitch + Math.copySign(chargeStationAlignFFMpS, pitch) : 0;
        if(Math.abs(fwd ? roll : pitch) < 8) forward = strafe = 0;
        drivetrain.drive(forward, strafe, 0);
    }

    @Override
    public boolean isFinished() {
        lastTime = Math.abs(fwd ? getRoll() : getPitch() /* Select which axis to use based on the direction of alignment */) < chargeStationAlignToleranceDeg ? // If the robot is aligned
                    lastTime == -1 ? // Set the last time to the current time if it hasn't been set yet
                        System.currentTimeMillis() :
                        lastTime : // else NOP
                    -1; // Reset the last time
        // The effect is that lastTime contains the time at which the robot was first aligned, so we can use it as a timer for how long the robot has been aligned

        return System.currentTimeMillis() - lastTime > chargeStationAlignTime && lastTime != -1;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            drivetrain.drive(fwd ? 0 : wheelTurnDriveSpeed, fwd ? wheelTurnDriveSpeed : 0, 0); // Turn the wheels to be perpendicular to the charging station
        }
        drivetrain.stop();
        drivetrain.setFieldOriented(fieldOriented);
    }

    private double getPitch() {
        return drivetrain.getPitch() - drivetrain.initPitch;
    }

    private double getRoll() {
        return drivetrain.getRoll() - drivetrain.initRoll;
    }

}
