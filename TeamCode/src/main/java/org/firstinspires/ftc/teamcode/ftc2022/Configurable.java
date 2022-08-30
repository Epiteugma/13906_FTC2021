package org.firstinspires.ftc.teamcode.ftc2022;

//import com.acmerobotics.dashboard.config.Config;
//
//@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static int armLowPosition = -525;
    public static int armMidPosition = -1150;
    public static int armHighPosition = -1850;

    public static int distanceToShippingHubRedLow = 50;
    public static int distanceToShippingHubRedMid = 50;
    public static int distanceToShippingHubRedHigh = 56;

    public static int distanceToShippingHubBlueLow = 46;
    public static int distanceToShippingHubBlueMid = 43;
    public static int distanceToShippingHubBlueHigh = 40;

    public static int distanceFromBackWallBlue = 45;
    public static int distanceFromBackWallRed = 50;

    public static double disposeLowSpeed = 0.6;
    public static double disposeMidSpeed = 0.7;
    public static double disposeHighSpeed = 0.9;

    public static double duckSpinnerPowerRed = 0.27;
    public static double duckSpinnerPowerBlue = 0.23;
    public static double duckSpinnerPowerTeleop = 0.38;


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
