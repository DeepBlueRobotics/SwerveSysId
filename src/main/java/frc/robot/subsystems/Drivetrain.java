package frc.robot.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import org.carlmontrobotics.lib199.swerve.SwerveModule.ModuleType;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax flDrive, frDrive, blDrive, brDrive, flTurn, frTurn, blTurn, brTurn;
    private final CANCoder flEncoder, frEncoder, blEncoder, brEncoder;

    private final CANSparkMax[] driveMotors, turnMotors;
    private final CANCoder[] encoders;
    private final SwerveModule[] modules; // Just used for turning

    private final PIDController[] turnControllers = {
        new PIDController(Constants.turnKp, 0, 0),
        new PIDController(Constants.turnKp, 0, 0),
        new PIDController(Constants.turnKp, 0, 0),
        new PIDController(Constants.turnKp, 0, 0)
    };

    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(Constants.turnKs, Constants.turnKv, 0);

    private boolean turnLock = false;

    public Drivetrain() {
        flDrive = MotorControllerFactory.createSparkMax(Constants.driveFrontLeftPort, TemperatureLimit.NEO);
        frDrive = MotorControllerFactory.createSparkMax(Constants.driveFrontRightPort, TemperatureLimit.NEO);
        blDrive = MotorControllerFactory.createSparkMax(Constants.driveBackLeftPort, TemperatureLimit.NEO);
        brDrive = MotorControllerFactory.createSparkMax(Constants.driveBackRightPort, TemperatureLimit.NEO);

        flTurn = MotorControllerFactory.createSparkMax(Constants.turnFrontLeftPort, TemperatureLimit.NEO);
        frTurn = MotorControllerFactory.createSparkMax(Constants.turnFrontRightPort, TemperatureLimit.NEO);
        blTurn = MotorControllerFactory.createSparkMax(Constants.turnBackLeftPort, TemperatureLimit.NEO);
        brTurn = MotorControllerFactory.createSparkMax(Constants.turnBackRightPort, TemperatureLimit.NEO);

        flDrive.setInverted(Constants.driveInversion[0]);
        frDrive.setInverted(Constants.driveInversion[1]);
        blDrive.setInverted(Constants.driveInversion[2]);
        brDrive.setInverted(Constants.driveInversion[3]);

        flTurn.setInverted(Constants.turnInversion[0]);
        frTurn.setInverted(Constants.turnInversion[1]);
        blTurn.setInverted(Constants.turnInversion[2]);
        brTurn.setInverted(Constants.turnInversion[3]);

        flEncoder = new CANCoder(Constants.flEncoderPort);
        frEncoder = new CANCoder(Constants.frEncoderPort);
        blEncoder = new CANCoder(Constants.blEncoderPort);
        brEncoder = new CANCoder(Constants.brEncoderPort);

        driveMotors = new CANSparkMax[] {flDrive, frDrive, blDrive, brDrive};
        turnMotors = new CANSparkMax[] {flTurn, frTurn, blTurn, brTurn};
        encoders = new CANCoder[] {flEncoder, frEncoder, blEncoder, brEncoder};

        double positionFactor = Constants.wheelDiameterMeters * Math.PI / Constants.driveGearing;

        for(CANSparkMax motor : driveMotors) {
            motor.getEncoder().setPositionConversionFactor(positionFactor);
            motor.getEncoder().setVelocityConversionFactor(positionFactor / 60);
        }

        for(CANSparkMax motor : turnMotors) {
            motor.getEncoder().setPositionConversionFactor(Constants.turnGearing * 360);
            motor.getEncoder().setVelocityConversionFactor(Constants.turnGearing * 360 / 60);
        }

        for(CANCoder encoder : encoders) encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        for(PIDController controller : turnControllers) {
            controller.setTolerance(Constants.turnTolerance);
            controller.enableContinuousInput(-180, 180);
        }

        modules = new SwerveModule[turnMotors.length];
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(Constants.swerveConfig, ModuleType.values()[i],
                MotorErrors.createDummySparkMax(),
                turnMotors[i],
                encoders[i], i,
                () -> 0f, () -> 0f);
        }
    }

    public void driveTurnMotor(ModuleType module, double voltage) {
        turnMotors[module.ordinal()].setVoltage(voltage);
        for(int i = 0; i < turnMotors.length; i++) if(i != module.ordinal()) turnMotors[i].setVoltage(0);
        for(CANSparkMax motor : driveMotors) motor.setVoltage(0);
    }

    public void driveRobot(double voltage, double direction) {
        boolean turnReady = true;
        for(int i = 0; i < turnMotors.length; i++) {
            double target = direction + Constants.turnZero[i];
            modules[i].move(0.000001, direction);
            if(MathUtil.inputModulus(target - encoders[i].getAbsolutePosition(), -180, 180) > Constants.turnTolerance) {
                turnReady = false;
            }
            modules[i].periodic();
        }
        turnLock |= turnReady;
        if(turnLock) for(CANSparkMax motor : driveMotors) motor.set(voltage);
    }

    public void resetTurnLock() {
        turnLock = false;
    }

    public void resetEncoders() {
        for(CANSparkMax motor : driveMotors) motor.getEncoder().setPosition(0);
        for(CANSparkMax motor : turnMotors) motor.getEncoder().setPosition(0);
    }

    public boolean getTurnLock() {
        return turnLock;
    }

    public double getEncoderPosition(ModuleType module, boolean drive) {
        return drive ? driveMotors[module.ordinal()].getEncoder().getPosition() : turnMotors[module.ordinal()].getEncoder().getPosition();
    }

    public double getEncoderVelocity(ModuleType module, boolean drive) {
        return drive ? driveMotors[module.ordinal()].getEncoder().getVelocity() : turnMotors[module.ordinal()].getEncoder().getVelocity();
    }

    public double getMotorVoltage(ModuleType module, boolean drive) {
        return getMotorVoltage(drive ? driveMotors[module.ordinal()] : turnMotors[module.ordinal()]);
    }

    public double getMotorVoltage(CANSparkMax motor) {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

}
