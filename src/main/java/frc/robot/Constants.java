// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int driveFrontLeftPort = 10;
    public static final int driveFrontRightPort = 2;
    public static final int driveBackLeftPort = 11;
    public static final int driveBackRightPort = 17;

    public static final int turnFrontLeftPort = 9;
    public static final int turnFrontRightPort = 1;
    public static final int turnBackLeftPort = 12;
    public static final int turnBackRightPort = 18;

    public static final boolean[] driveInversion = {false, false, false, false};
    public static final boolean[] turnInversion = {true, true, true, true};

    public static final int flEncoderPort = 1;
    public static final int frEncoderPort = 2;
    public static final int blEncoderPort = 3;
    public static final int brEncoderPort = 4;

    public static final double[] turnZero = {-72.861, 47.549, -178.505, 66.885};

    public static final double driveGearing = 6.75;
    public static final double turnGearing = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65;

    public static final double turnKs = 0.02;
    public static final double turnKv = 0.00463;
    public static final double turnKp = 0.00374;
    public static final double turnTolerance = 3 / 4096;

    public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, 0.5, 0.5 * 9.8 * 2, new double[4], new double[4], new double[4], new double[4], new double[4], new double[4], new double[4], new double[4], new double[4], new double[] {turnKp, turnKp, turnKp, turnKp}, new double[4], new double[4], new double[] {turnKs, turnKs, turnKs, turnKs}, new double[] {turnKv, turnKv, turnKv, turnKv}, new double[4], turnZero, driveInversion, new boolean[] {false, false, false, false}, 0, turnInversion);

}
