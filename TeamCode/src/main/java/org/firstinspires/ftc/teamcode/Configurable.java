package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static int armLowPosition = -450;
    public static int armMidPosition = -1100;
    public static int armHighPosition = -1800;

    public static double disposeLowSpeed = 0.5;
    public static double disposeMidSpeed = 0.65;
    public static double disposeHighSpeed = 0.7;

    public static int disposeTicks = 576;

    public static double duckSpinnerPowerRed = 0.25;
    public static double duckSpinnerPowerBlue = -0.25;

    public static int duckSpinnerTicks = 1500;

    public static double driveGearRatio = 14/2;

    public static double armTickPerRev = 1120.0;

    // TODO: adjust gain (almost done)
    public static double driveGain = 0.05;
    public static double strafeGain = 0.125;
    public static double turnGain = 0.1;

    // Wheel dimensions and distances
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 7.5;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
    public static double turnCircumference = 2 * Math.PI * centerToWheel;
    public static double normalFullPowerVelocity = 2700;

    // Barriers distance from the wall
    public static double afterBarriers = 55;
    public static double beforeBarriers = 819;

    // cargoDetector heights and measurements
    public double cubeHeight= 5.08;
    public double ballHeight = 6.99;
    public double duckHeight = 5.4;
    public double collectorBoxHeight = 15;
}
