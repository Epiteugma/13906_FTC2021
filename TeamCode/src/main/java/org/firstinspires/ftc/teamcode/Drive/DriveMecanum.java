package org.firstinspires.ftc.teamcode.Drive;

// Navigation and IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Sensors , Motors and Opmode
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//FTCLib
import com.arcrobotics.ftclib.gamepad.*;
import com.arcrobotics.ftclib.hardware.*;
import com.arcrobotics.ftclib.hardware.motors.*;

@TeleOp(name = "FTC 2022 Drive (Mecanum)", group = "FTC22")
public class DriveMecanum extends LinearOpMode {

    // cargoDetector
    String detectedCargo = "None";
    String prevDetectedCargo = "None";
    double cubeHeight= 5.08;
    double ballHeight = 6.99;
    double duckHeight = 5.4;
    double currentDistance = 0;
    double collectorBoxHeight = 0;
    SensorRevTOFDistance cargoDetector = null;

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        // Motors, servos initialization
        SensorRevTOFDistance cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        Motor arm = new Motor(hardwareMap, "arm");
        Motor collector = new Motor(hardwareMap, "collector");
        ServoEx capper= new SimpleServo(hardwareMap, "capper",0,90);
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");

        // Fix all the directions of the motors.
        frontRight.setInverted(true);
        backRight.setInverted(true);

        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);
        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);

        // IMU init.
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        // IMU remapping axis
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        // Gamepads init
        GamepadEx gamepad1 = new GamepadEx(this.gamepad1);
        GamepadEx gamepad2 = new GamepadEx(this.gamepad2);
        // Meccanum drivebase; Pass the motor objects
        MecanumDrive drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Current Free buttons:
        // Turbo

        // Custom controller keymaping
        GamepadKeys.Button CROSS = GamepadKeys.Button.A;
        GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
        GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
        GamepadKeys.Button SQUARE = GamepadKeys.Button.X;

        // IMU
        double heading;
        // power and constants
        double sidepower;
        double forwardpower;
        double turnpower;
        double duckSpinnersPower = 0;
        // initial box size
        int collectorTicksPerRevolution = 0;
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // power factors
        double multiplier = 0.6;
        double globalpowerfactor = 1.0;
        // Arm and positions
        //TODO: Calibrate the ticks needed for each of the 3 levels
        double armPower = 0.6;
        double lowPosition = 100;
        double midPosition = 200;
        double highPosition = 300;
        double lastClawPosition = arm.getCurrentPosition();
        double armTickPerRev = 0;

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

            if(gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                globalpowerfactor += 0.1;
            }
            else if(gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                globalpowerfactor -= 0.1;
            }

            // Calculate Mecanum Powers for field-centric drive
            drivetrain.driveFieldCentric(sidepower, forwardpower, turnpower, heading);

            // Arm predifined positions
            arm.setPositionTolerance(2);
            if (gamepad2.wasJustPressed(CROSS)){
                if (lowPosition > lastClawPosition) {
                    lowPosition = lowPosition + lastClawPosition;
                }
                else {
                    lowPosition = lowPosition - lastClawPosition;
                }
                arm.setTargetPosition((int) (lowPosition * armTickPerRev));
                lastClawPosition = lowPosition;
                }
            else if (gamepad2.wasJustPressed(CIRCLE)){
                if (midPosition > lastClawPosition) {
                    midPosition = midPosition + lastClawPosition;
                }
                else {
                    midPosition = midPosition - lastClawPosition;
                }
                arm.setTargetPosition((int) (midPosition * armTickPerRev));
                lastClawPosition = midPosition;
                }
            else if (gamepad2.wasJustPressed(TRIANGLE)){
                // No need to check if above it will never be above the highest possible position
                highPosition = highPosition - lastClawPosition;
                arm.setTargetPosition((int) (highPosition * armTickPerRev));
                lastClawPosition = highPosition;
            }
//            else if(DOWN){
//                arm.setTargetPosition((int) (-lastClawPosition * armTickPerRev));
//            }
            while (!collector.atTargetPosition()) {
                arm.set(armPower);
            }

            // TODO: fix arm and capper.
            // Arm up DPAD_UP
            if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                arm.setRunMode(Motor.RunMode.RawPower);
                arm.set(0.5);
            }
            // Arm down DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                arm.setRunMode(Motor.RunMode.RawPower);
                arm.set(0.5);
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                capper.rotateByAngle(1);
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                capper.rotateByAngle(-1);
            } else {
                arm.set(0);
            }

            // INTAKE CODE
            if(gamepad2.getButton(GamepadKeys.Button.START)) {
                if(isCollectorActive && collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } 
            else if(gamepad2.getButton(GamepadKeys.Button.BACK)) {
                if(isCollectorActive && !collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = false;
                }
            }

            // DUCK SPINNER CODE
            if(gamepad1.wasJustPressed(SQUARE)) {
                duckSpinnersPower = duckSpinnersPower == multiplier-0.25 ? 0 : multiplier-0.25;
            }
            duckSpinners.set(duckSpinnersPower);

            if(isCollectorActive) {
                collector.set(collectorDirection ? 1 : -multiplier);
            } 
            else {
                collector.set(0);
            }

            // Telemetry
            detectedCargo = cargoDetection();
            if (!prevDetectedCargo.equals(detectedCargo) && prevDetectedCargo.equals("None")) {
                // Beta stop the intake when freight is collected and vibrate the drivers' controllers to make them aware
                collector.stopMotor();
                // Vibrate only the right part (means cube or duck)
                if (detectedCargo.equals("Cuber OR Duck")) {
                    this.gamepad1.rumble(0,1,1000);
                    this.gamepad2.rumble(0,1,1000);
                }
                // Vibrate only the left part (means ball)
                if (detectedCargo.equals("Ball")) {
                    this.gamepad1.rumble(1,0,1000);
                    this.gamepad2.rumble(1,0,1000);
                }
            }
            prevDetectedCargo = detectedCargo;
            telemetry.addData("Detected Cargo: ", detectedCargo);
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            telemetry.addData("frontRight: ", frontRight.get());
            telemetry.addData("frontLeft: ", frontLeft.get());
            telemetry.addData("backRight: ", backRight.get());
            telemetry.addData("backLeft: ", backLeft.get());
            telemetry.addData("Arm: ", arm.get());
            telemetry.addData("Collector: ", collector.get());
            telemetry.update();
        }
    }
    private String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        currentDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (collectorBoxHeight - cubeHeight < collectorBoxHeight - currentDistance) {
            return "Ball";
        }
        else if(collectorBoxHeight - ballHeight < collectorBoxHeight - currentDistance) {
            return "Cube OR Duck";
        }
        else {
            return "None";
        }
    }
}
