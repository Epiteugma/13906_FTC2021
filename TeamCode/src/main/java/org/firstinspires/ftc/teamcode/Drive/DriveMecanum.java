package org.firstinspires.ftc.teamcode.Drive;

// Navigation and IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Sensors , Motors and Opmode
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//FTCLib
import com.arcrobotics.ftclib.gamepad.*;
import com.arcrobotics.ftclib.hardware.*;
import com.arcrobotics.ftclib.hardware.motors.*;

@TeleOp(name = "FTC 2022 Drive (Mecanum) Final", group = "FTC22")
public class DriveMecanum extends LinearOpMode {


    // duckSpinners
    public boolean duckSpinnersEnabled = false;
    // power factors
    public double globalpowerfactor = 1.0;
    // Arm and positions
    //TODO: Calibrate the ticks needed for each of the 3 levels
    public double armPositionalPower = 0.0;
    public double armPower = Configurable.armPower;
    public int lowPosition = Configurable.lowPosition;
    public int midPosition = Configurable.midPosition;
    public int highPosition = Configurable.highPosition;

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

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors, servos initialization
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        duckSpinner1.setRunMode(Motor.RunMode.VelocityControl);
        duckSpinner2.setRunMode(Motor.RunMode.VelocityControl);
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        Motor arm = new Motor(hardwareMap, "arm");
        Motor collector = new Motor(hardwareMap, "collector");
        CRServo capper= new CRServo(hardwareMap, "capper");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");

        frontRight.setInverted(true);

        arm.resetEncoder();

        duckSpinners.setRunMode(Motor.RunMode.RawPower);

        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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
        // Collector

        int lastClawPosition = arm.getCurrentPosition();
        arm.resetEncoder();
        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        while (opModeIsActive()) {
            // Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = imu.getHeading();

            sidepower = gamepad1.getLeftX() * globalpowerfactor;
            forwardpower = gamepad1.getLeftY() * globalpowerfactor;
            turnpower = gamepad1.getRightX() * globalpowerfactor;

            if(gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                globalpowerfactor = 1.0;
            }
            else if(gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER) || gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                globalpowerfactor = 0.35;
            }

            // Calculate Mecanum Powers for field-centric drive
            drivetrain.driveFieldCentric(sidepower, forwardpower, turnpower, heading);

            // Arm predefined positions
            arm.setRunMode(Motor.RunMode.PositionControl);
            arm.setPositionTolerance(40); // has to be close to the teeth of the small gear

            // Arm up DPAD_UP
            if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                if (arm.getCurrentPosition() >= -1800) {
                    arm.setRunMode(Motor.RunMode.RawPower);
                    arm.set(-armPower);
                }
            }
            // Arm down DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                if (arm.getCurrentPosition() <= 0) {
                    arm.setRunMode(Motor.RunMode.RawPower);
                    arm.set(armPower);
                }
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                capper.set(-0.2);
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                capper.set(0.2);
            }
            else {
                capper.stopMotor();
                if (gamepad2.getButton(CROSS)) {
                        lastClawPosition = lowPosition;
                        armPositionalPower = 0.3;
                        arm.setTargetPosition(lastClawPosition);
                        while (!arm.atTargetPosition()) {
                            arm.set(armPositionalPower);
                        }
                    }
                else if (gamepad2.getButton(CIRCLE)) {
                    lastClawPosition = midPosition;
                    armPositionalPower = 0.2;
                    arm.setTargetPosition(lastClawPosition);
                    while (!arm.atTargetPosition()) {
                        arm.set(armPositionalPower);
                    }
                }
                else if (gamepad2.getButton(TRIANGLE)) {
                    lastClawPosition = highPosition;
                    armPositionalPower = 0.1;
                    arm.setTargetPosition(lastClawPosition);
                    while (!arm.atTargetPosition()) {
                        arm.set(armPositionalPower);
                    }
                }
                // ARM KEEP POSITION!!!
                else{
                    lastClawPosition = arm.getCurrentPosition();
                    armPositionalPower = 0.1;
                    arm.setTargetPosition(lastClawPosition);
                    //armPositionalPower = +(+lastClawPosition / 275.0 /10.0);
                    if (!arm.atTargetPosition()) {
                        arm.set(armPositionalPower);
                    }
                }
            }

            // INTAKE CODE
            if(gamepad2.isDown(GamepadKeys.Button.BACK)) {
                collector.set(+globalpowerfactor + 0.3);
            } 
            else if(gamepad2.isDown(GamepadKeys.Button.START)) {
                collector.set(-globalpowerfactor - 0.3);
            }
            else {
                collector.stopMotor();
            }

            // DUCK SPINNER CODE
            if(gamepad1.getButton(SQUARE)) {
                if (!duckSpinnersEnabled) {
                    duckSpinnersEnabled = true;
                    duckSpinners.set(-0.1);
                }
                else {
                    duckSpinnersEnabled = false;
                    duckSpinners.stopMotor();
                }
            }

            // Telemetry
            detectedCargo = cargoDetection();
            if (!detectedCargo.equals("None") && prevDetectedCargo.equals("None")) {
                // Beta stop the intake when freight is collected and vibrate the drivers' controllers to make them aware
                collector.stopMotor();
                // Vibrate only the right part (means cube or duck)
                if (detectedCargo.equals("Cuber OR Duck")) {
                    this.gamepad1.rumble(0,1,1000);
                    this.gamepad2.rumble(0,1,1000);
                }
                // Vibrate only the left part (means ball)
                else if (detectedCargo.equals("Ball")) {
                    this.gamepad1.rumble(1,0,1000);
                    this.gamepad2.rumble(1,0,1000);
                }
            }

            telemetry.addData("Probably Detected Cargo: ", detectedCargo);
            telemetry.addData("Probably Prev Detected Cargo: ", prevDetectedCargo);
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            telemetry.addData("frontRight: ", frontRight.get());
            telemetry.addData("frontLeft: ", frontLeft.get());
            telemetry.addData("backRight: ", backRight.get());
            telemetry.addData("backLeft: ", backLeft.get());
            telemetry.addData("Arm: ", arm.get());
            telemetry.addData("Arm ticks: ",lastClawPosition);
            telemetry.addData("Arm positional power: ",armPositionalPower);
            telemetry.addData("Collector: ", collector.get());
            telemetry.addData("DucksSpinners power: ", duckSpinners.get());
            telemetry.addData("Initial Box Height: ", collectorBoxHeight);
            telemetry.addData("Height of cargo: ", collectorBoxHeight - currentCargoDistance);
            telemetry.update();
            prevDetectedCargo = detectedCargo;
        }
    }
}