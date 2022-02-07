package org.firstinspires.ftc.teamcode.drive;

// Navigation and IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Sensors , Motors and Opmode
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

//FTCLib
import com.arcrobotics.ftclib.gamepad.*;
import com.arcrobotics.ftclib.hardware.*;
import com.arcrobotics.ftclib.hardware.motors.*;
import com.qualcomm.robotcore.hardware.Servo;

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

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        backRight.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        frontLeft.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        frontRight.setRunMode(Motor.RunMode.RUN_USING_ENCODER);

        // IMU init.
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibackRightation.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init(params);
        // IMU remapping axis
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        // Gamepads init
        GamepadEx gamepad1 = new GamepadEx(this.gamepad1);
        GamepadEx gamepad2 = new GamepadEx(this.gamepad2);
        // Meccanum drivebase; Pass the motor objects
        MecanumDrive drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Custom controller keymaping
        GamepadKeys.Button CROSS = GamepadKeys.Button.A;
        GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
        GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
        GamepadKeys.Button SQUARE = GamepadKeys.Button.X;

        // power and constants
        double duckSpinnersPower = 0;
        // initial positions
        double capperPosition = capper.getPosition();
        double armPosition = arm.getPosition();
        // initial box size
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // power factors
        double multiplier = 0.6;
        double globalpowerfactor = 1.0;

        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        while (opModeIsActive()) {
            // Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = imu.getHeading();

            double sidepower = gamepad1.getLeftX() * globalpowerfactor;
            double forwardpower = gamepad1.getLeftY() * globalpowerfactor;
            double turnpower = gamepad1.getRightX() * globalpowerfactor;

            if(gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                globalpowerfactor += 0.1;
            }
            else if(gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                globalpowerfactor -= 0.1;
            }

            // Calculate Mecanum Powers for field-centric drive
            drivetrain.driveFieldCentric(sidepower, forwardpower, turnpower, heading);

            // Arm up DPAD_UP
            if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                armPosition += 0.1;
                arm.setPosition(armPosition);
                arm.setRunMode(Motor.RunMode.RUN_TO_POSITION);
            }
            // Arm down DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                armPosition -= 0.1;
                arm.setPosition(armPosition);
                arm.setRunMode(Motor.RunMode.RUN_TO_POSITION);
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                capperPosition += 0.1;
                capper.setPosition(capperPosition);
                capper.setRunMode(ServoEx.RunMode.RUN_TO_POSITION);
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                capperPosition -= 0.1;
                capper.setPosition(capperPosition);
                capper.setRunMode(ServoEx.RunMode.RUN_TO_POSITION);
            }


            // INTAKE CODE
            if(gamepad2.wasJustPressed(CROSS)) {
                if(isCollectorActive && collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } else if(gamepad2.wasJustPressed(TRIANGLE)) {
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
                collector.setPower(collectorDirection ? 1 : -multiplier);
            } 
            else {
                collector.setPower(0);
            }

            // Telemetry
            detectedCargo = cargoDetection();
            if (prevDetectedCargo != detectedCargo && prevDetectedCargo == "None") {
                this.gamepad1.rumble(1,1,1000);
                this.gamepad2.rumble(1,1,1000);
            }
            prevDetectedCargo = detectedCargo;
            telemetry.addData("Detected Cargo", detectedCargo);
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            telemetry.addData("frontRight: ", -(forwardpower + sidepower - turnpower));
            telemetry.addData("frontLeft: ", (forwardpower - sidepower + turnpower));
            telemetry.addData("backRight: ", -(forwardpower - sidepower - turnpower));
            telemetry.addData("backLeft: ", (forwardpower + sidepower + turnpower));
            telemetry.addData("Arm :", arm.getPower());
            telemetry.addData("Collector: ", collector.getPower());
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
