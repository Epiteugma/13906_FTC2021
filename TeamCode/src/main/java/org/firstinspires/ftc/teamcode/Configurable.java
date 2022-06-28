package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static int armLowPosition = -525;
    public static int armMidPosition = -1150;
    public static int armHighPosition = -1890;

    public static int distanceToShippingHubRedLow = 46;
    public static int distanceToShippingHubRedMid = 46;
    public static int distanceToShippingHubRedHigh = 46;

    public static int distanceToShippingHubBlueLow = 36;
    public static int distanceToShippingHubBlueMid = 36;
    public static int distanceToShippingHubBlueHigh = 36;

    public static int distancefromBackWallBlue = 45;
    public static int distancefromBackWallRed = 50;

    public static double disposeLowSpeed = 0.6;
    public static double disposeMidSpeed = 0.7;
    public static double disposeHighSpeed = 0.9;

    public static double duckSpinnerPower = 0.23;

    public static int duckSpinnerTicks = 1000;

    public static double driveGearRatio = 14.0/2.0;

    // Wheel dimensions and distances
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 7.5;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
}
