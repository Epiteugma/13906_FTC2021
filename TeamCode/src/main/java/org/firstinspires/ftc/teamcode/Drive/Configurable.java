package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurable {
    public static double armPower = 0.75;
    // Arm and positions
    //TODO: Calibrate the ticks needed for each of the 3 levels
    public static int lowPosition = -370;
    public static int midPosition = -1000;
    public static int highPosition = -1800;
    public static int capperLowLimit = 200;
    public static int capperHighLimit = 300;
    // public static double armPositionalPower = 0;
}
