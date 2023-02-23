// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int flDrivePort = 8;
    public static final int frDrivePort = 13;
    public static final int blDrivePort = 5;
    public static final int brDrivePort = 11;
    public static final int flTurnPort = 7;
    public static final int frTurnPort = 14;
    public static final int blTurnPort = 6;
    public static final int brTurnPort = 12;

    public static final boolean[] driveInversion = {false, false, false, false};
    public static final boolean[] turnInversion = {true, true, true, true};

    public static final int flEncoderPort = 1;
    public static final int frEncoderPort = 2;
    public static final int blEncoderPort = 3;
    public static final int brEncoderPort = 4;

    public static final double[] turnZero = {85.7812, 85.0782 , -96.9433, -162.9492};

    public static final double driveGearing = 6.75;
    public static final double turnGearing = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65;

    public static final double turnKp = 0.00374;
    public static final double turnTolerance = 3 / 4096;

}
