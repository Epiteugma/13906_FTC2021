package org.firstinspires.ftc.teamcode.Drive;

// Navigation and IMU
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Sensors , Motors and Opmode
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//FTCLib
import com.arcrobotics.ftclib.gamepad.*;
import com.arcrobotics.ftclib.hardware.*;
import com.arcrobotics.ftclib.hardware.motors.*;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOp", group = "FTC22Tele")
public class DriveMecanum extends LinearOpMode {

    // power factors
    public double globalpowerfactor = 1.0;

    public double duckSpinnersPower = 0;
    public double lastDuckSpinnersPower = 0.45;

    public boolean armOverride = false;
    public double armPositionalPower = 0;
    public double armPower = Configurable.armPower;
    public int lowPosition = Configurable.lowPosition;
    public int midPosition = Configurable.midPosition;
    public int highPosition = Configurable.highPosition;
    public int capperHighLimit = Configurable.capperHighLimit;
    public int capperLowLimit = Configurable.capperLowLimit;

    // IMU
    public double heading;
    // power and constants
    public double sidepower;
    public double forwardpower;
    public double turnpower;

    // cargoDetector
    public String detectedCargo = "None";
    public String prevDetectedCargo = "None";
    public double cubeHeight= 5.08;
    public double ballHeight = 6.99;
    public double duckHeight = 5.4;
    public double currentCargoDistance = 0;
    public double collectorBoxHeight = 0;
    SensorRevTOFDistance cargoDetector = null;

    public String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        currentCargoDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (3.45 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 7.5) {
            return "Ball";
        }
        else if(1 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 3.45) {
            return "Cube OR Duck";
        }
        else {
            return "None";
        }
    }

    // Colored telemetry based on given hex code like html
    public void cTelemetry(String Tag,String tagColor, String color,String msg){
        if (tagColor.equals("def")){
            tagColor = "#e37b29";
        }
        msg = msg.replaceAll("<","&lt").replaceAll(">","&gt;").replaceAll(" ","&nbsp;").replaceAll("\"","&quot").replaceAll("'","&apos;");
        telemetry.addData(String.format("<span style=\"color:%s\">%s</span>",tagColor,Tag), String.format("<span style=\"color:%s\">%s</span>",color,msg));
    }

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        double duckSpinnerPrevTime = 0;

        cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors, servos initialization
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        duckSpinner1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        duckSpinner2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        duckSpinners.setRunMode(Motor.RunMode.RawPower);
        Motor arm = new Motor(hardwareMap, "arm");
        arm.resetEncoder();
        Motor collector = new Motor(hardwareMap, "collector");
        CRServo capper = new CRServo(hardwareMap, "capper");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        frontRight.setInverted(true);
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");

        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // IMU init.
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // Gamepads init
        GamepadEx gamepad1 = new GamepadEx(this.gamepad1);
        GamepadEx gamepad2 = new GamepadEx(this.gamepad2);
        // Mecanum drivebase (Pass the motor objects)
        MecanumDrive drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Current Free buttons:
        // Turbo

        // Custom controller keymaping
        GamepadKeys.Button CROSS = GamepadKeys.Button.A;
        GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
        GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
        GamepadKeys.Button SQUARE = GamepadKeys.Button.X;

        // initial box size
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);

        int lastClawPosition = arm.getCurrentPosition();
        arm.resetEncoder();

        // Colored telemetry (basically html)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        TouchSensor armTouch1 = hardwareMap.get(TouchSensor.class, "armTouch1");
        TouchSensor armTouch2 = hardwareMap.get(TouchSensor.class, "armTouch2");

        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        double globalPowerPrevTime1 = 0;
        double globalPowerPrevTime2 = 0;

        while (opModeIsActive()) {
            // Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = imu.getHeading();

            // Driver vibration intercommunication
            if(gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 || gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
              this.gamepad1.rumble(0,1,750);
              this.gamepad2.rumble(1,1,750);
            }
            // GlobalPowerFactor manipulation
            if(gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && System.currentTimeMillis() > globalPowerPrevTime1+200) {
                globalPowerPrevTime1 = System.currentTimeMillis();
                globalpowerfactor += 0.2;
            }
            if(gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && System.currentTimeMillis() > globalPowerPrevTime2+200) {
                globalPowerPrevTime2 = System.currentTimeMillis();
                globalpowerfactor += 0.2;
            }
            if(gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER) && System.currentTimeMillis() > globalPowerPrevTime1+200) {
                globalPowerPrevTime1 = System.currentTimeMillis();
                globalpowerfactor -= 0.2;
            }
            if(gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER) && System.currentTimeMillis() > globalPowerPrevTime2+200) {
                globalPowerPrevTime2 = System.currentTimeMillis();
                globalpowerfactor -= 0.2;
            }
            if (globalpowerfactor >= 1){
                globalpowerfactor = 1;
            }
            else if (globalpowerfactor <= 0.4){
                globalpowerfactor = 0.4;
            }

            sidepower = gamepad1.getLeftX() * globalpowerfactor;
            forwardpower = gamepad1.getLeftY() * globalpowerfactor;
            turnpower = gamepad1.getRightX() * globalpowerfactor;
            drivetrain.driveRobotCentric(sidepower, forwardpower, turnpower);

            // Arm predefined positions
            arm.setRunMode(Motor.RunMode.PositionControl);
            arm.setPositionTolerance(40); // has to be close to the teeth of the small gear

            // capper Limit checker
//            capper.setRunMode(Motor.RunMode.PositionControl);
//            capper.setPositionTolerance(0);
//            int capperCurrentPosition = capper.getCurrentPosition();

            // Arm up DPAD_UP
            if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                if (arm.getCurrentPosition() >= -1850 || armOverride) {
                    arm.setRunMode(Motor.RunMode.RawPower);
                    arm.set(-armPower);
                }
            }
            // Arm down DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                if (arm.getCurrentPosition() <= 0 || armOverride) {
                    arm.setRunMode(Motor.RunMode.RawPower);
                    arm.set(armPower);
                }
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.getButton(GamepadKeys.Button.START)) {
                // if ( capperCurrentPosition < capperHighLimit){
                capper.setRunMode(Motor.RunMode.RawPower);
                capper.set(-0.5);
                // }
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
                // if (capperCurrentPosition > capperLowLimit){
                capper.setRunMode(Motor.RunMode.RawPower);
                capper.set(gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) /2.15);
                // }
            }
            else {
                capper.stopMotor();
                if (gamepad2.getButton(CROSS)) {
                        lastClawPosition = lowPosition;
                        armPositionalPower = 0.45;
                        arm.setTargetPosition(lastClawPosition);
                        while (!arm.atTargetPosition()) {
                            sidepower = gamepad1.getLeftX() * globalpowerfactor;
                            forwardpower = gamepad1.getLeftY() * globalpowerfactor;
                            turnpower = gamepad1.getRightX() * globalpowerfactor;
                            drivetrain.driveRobotCentric(sidepower, forwardpower, turnpower);
                            arm.set(armPositionalPower);
                        }
                    }
                else if (gamepad2.getButton(CIRCLE)) {
                    lastClawPosition = midPosition;
                    armPositionalPower = 0.4;
                    arm.setTargetPosition(lastClawPosition);
                    while (!arm.atTargetPosition()) {
                        sidepower = gamepad1.getLeftX() * globalpowerfactor;
                        forwardpower = gamepad1.getLeftY() * globalpowerfactor;
                        turnpower = gamepad1.getRightX() * globalpowerfactor;
                        drivetrain.driveRobotCentric(sidepower, forwardpower, turnpower);
                        arm.set(armPositionalPower);
                    }
                }
                else if (gamepad2.getButton(TRIANGLE)) {
                    lastClawPosition = highPosition;
                    armPositionalPower = 0.35;
                    arm.setTargetPosition(lastClawPosition);
                    while (!arm.atTargetPosition()) {
                        sidepower = gamepad1.getLeftX() * globalpowerfactor;
                        forwardpower = gamepad1.getLeftY() * globalpowerfactor;
                        turnpower = gamepad1.getRightX() * globalpowerfactor;
                        drivetrain.driveRobotCentric(sidepower, forwardpower, turnpower);
                        arm.set(armPositionalPower);
                    }
                }
                // ARM KEEP POSITION!!!
                else {
                    if (this.gamepad2.touchpad){
                        arm.resetEncoder();
                        armOverride = false;
                    }
                    lastClawPosition = arm.getCurrentPosition();
                    // OLD now proportional???
                    armPositionalPower = 0.09;
                    arm.setTargetPosition(lastClawPosition);
                    //armPositionalPower = +(+lastClawPosition / 450.0 /10.0);
                    if (!arm.atTargetPosition()) {
                        arm.set(armPositionalPower);
                    }
                }
            }

            if(gamepad2.getButton(SQUARE)) {
                if(gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) arm.set(-0.5);
                else if(gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) arm.set(0.5);
                else arm.set(0);
                armOverride = true;
                // arm.resetEncoder();
            }

            // INTAKE / COLLECTOR CODE
            // Collect
            if(gamepad2.isDown(GamepadKeys.Button.BACK)) {
                collector.set(+globalpowerfactor + 0.2);
            } 

            // Release/Throw
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
                    collector.set(-0.8 * gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                }
            // Stop collector (no action)
            else {
                collector.stopMotor();
            }

            // DUCK SPINNER CODE
            if (gamepad1.getButton((GamepadKeys.Button.DPAD_RIGHT))){
                duckSpinners.setInverted(true);
            }
            else if (gamepad1.getButton((GamepadKeys.Button.DPAD_LEFT))){
                duckSpinners.setInverted(false);
            }
            else if(gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
                duckSpinnersPower += 0.03;
            }
            else if(gamepad1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                duckSpinnersPower -= 0.03;
            }
            else if (gamepad1.getButton(SQUARE) && duckSpinnerPrevTime+250 <= System.currentTimeMillis()) {
                duckSpinnerPrevTime = System.currentTimeMillis();
                if (duckSpinners.get() != 0) {
                    lastDuckSpinnersPower = duckSpinnersPower;
                    duckSpinnersPower = 0;
                }
                else{
                    duckSpinnersPower = lastDuckSpinnersPower;
                }
            }
            duckSpinners.set(duckSpinnersPower);

            // Check touch sensors to reset arm encoder
            if (armTouch1.isPressed() || armTouch2.isPressed()){
                arm.resetEncoder();
                armOverride = false;
            }

            // FULL SPEED NAVIGATION
            if (gamepad1.isDown(TRIANGLE)){
              drivetrain.driveRobotCentric(0,1,0);
            }
            else if (gamepad1.isDown(CROSS)){
              drivetrain.driveRobotCentric(0,-1,0);
            }
//            else if (gamepad1.isDown(CIRCLE)){
//              drivetrain.driveRobotCentric(0,0,1);
//            }
//            else if (gamepad1.isDown(SQUARE)){
//              drivetrain.driveRobotCentric(0,0,-1);
//            }


            // Telemetry
            detectedCargo = cargoDetection();
            if ((detectedCargo.equals("Cuber OR Duck") || detectedCargo.equals("Ball")) && prevDetectedCargo.equals("None")) {
                // Beta stop the intake when freight is collected and vibrate the drivers' controllers to make them aware
                // collector.stopMotor();
                // Vibrate only the right part (means cube or duck)
                if (detectedCargo.equals("Cuber OR Duck")) {
                    this.gamepad1.rumble(0,1,1000);
                    this.gamepad2.rumble(0,1,1000);
                }
                // Vibrate only the left part (means ball)
                else {
                    this.gamepad1.rumble(1,0,1000);
                    this.gamepad2.rumble(1,0,1000);
                }
            }

            // New colored telemetry
            String blue = "#001dff";
            String red = "#EF3F49";
            String yellow = "#e7ff00";
            String green = "#11ff00";
            String orange = "#ff9900";

            cTelemetry("Probably Detected Cargo: ","def",orange, detectedCargo);
            cTelemetry("GlobalPowerFactor: ","def",red, String.valueOf(globalpowerfactor));
            cTelemetry("frontRight: ","def",red, String.valueOf(frontRight.get()));
            cTelemetry("frontLeft: ","def",red, String.valueOf(frontLeft.get()));
            cTelemetry("backRight: ","def",red , String.valueOf(backRight.get()));
            cTelemetry("backLeft: ","def",red, String.valueOf(backLeft.get()));
            cTelemetry("Arm: ","def",red , String.valueOf(arm.get()));
            cTelemetry("Arm ticks: ","def",green, String.valueOf(lastClawPosition));
            cTelemetry("Arm positional power: ","def",red, String.valueOf(armPositionalPower));
            cTelemetry("Collector: ","def",red, String.valueOf(collector.get()));
            cTelemetry("DucksSpinners power: ","def",red, String.valueOf(duckSpinners.get()));
            cTelemetry("DucksSpinners power variable: ","def",yellow, String.valueOf(duckSpinnersPower));
            cTelemetry("Initial Box Height: ","def",yellow, String.valueOf(collectorBoxHeight));
            cTelemetry("Height of cargo: ","def",yellow, String.valueOf(collectorBoxHeight - currentCargoDistance));
            cTelemetry("Probably Prev Detected Cargo: ","def",orange, prevDetectedCargo);
            cTelemetry("frontRight ticks: ","def",green, String.valueOf(frontRight.getCurrentPosition()));
            cTelemetry("frontLeft ticks: ","def",green, String.valueOf(frontLeft.getCurrentPosition()));
            cTelemetry("backRight ticks: ","def",green, String.valueOf(backRight.getCurrentPosition()));
            cTelemetry("backLeft ticks: ","def",green, String.valueOf(backLeft.getCurrentPosition()));
            telemetry.update();
            prevDetectedCargo = detectedCargo;
        }
    }
}