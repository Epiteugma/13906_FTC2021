package org.firstinspires.ftc.teamcode.drive;

// Navigation and IMU
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Sensors , Motors and Opmode
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//FTCLib
import com.arcrobotics.ftclib.gamepad;
import com.arcrobotics.ftclib.hardware;
import com.arcrobotics.ftclib.hardware.motors;

// Misc utils
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FTC 2022 Drive (Mecanum)", group = "FTC22")
public class DriveMecanum extends LinearOpMode {

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        // Motors, servos and IMU
        BNO055IMU IMU = hardwareMap.get(BNO055IMU.class, "imu");
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        Motor m_armClaw = new Motor(hardwareMap, "armClaw");
        Motor m_collector = new Motor(hardwareMap, "collector");
        // CRServo capper = hardwareMap.get(CRServo.class, "capper");
        Motor m_frontRight = new Motor(hardwareMap, "frontRight");
        Motor m_frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor m_backRight = new Motor(hardwareMap, "backRight");
        Motor m_backLeft = new Motor(hardwareMap, "backLeft");

        // grab the internal DcMotor object
        DcMotor frontRight = frontRight.motor;
        DcMotor frontLeft = frontLeft.motor;
        DcMotor backRight = m_backRight.motor;
        DcMotor backLeft = m_backLeft.motor;
        DcMotor armClaw = m_armClaw.motor;
        DcMotor collector = m_collector.motor;

        // Fix all the directions of the motors.
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init.
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibackRightationDataFile = "BNO055IMUCalibackRightation.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        // IMU remaping axis
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        // power constants
        double clawPower = 0;
        double duckSpinnersPower = 0;
        double capperPower = 0;
        // timers
        double prevTime = 0;
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // power factors
        double multiplier = 0.6;
        double globalpowerfactor = 1.0;
        // Gamepads init
        GamepadEx gamepad1 = new Gamepad(gamepad1);
        GamepadEx gamepad2 = new Gamepad(gamepad2);
        // Meccanum drivebase; Pass the motor objects not the DcMotor objects
        MecanumDrive drivetrain = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        while (opModeIsActive()) {
            Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double sidepower = gamepad1.getLeftX() * globalpowerfactor;
            double forwardpower = gamepad1.getLeftY() * globalpowerfactor;
            double turnpower = gamepad1.getRightY() * globalpowerfactor;

            if(gamepad1.wasJustPressed(GamepadKeys.RIGHT_BUMPER)) {
                globalpowerfactor += 0.1;
            }
            else if(gamepad1.wasJustPressed(GamepadKeys.LEFT_BUMPER)) {
                globalpowerfactor -= 0.1;
            }

            // Smooth out the power. Max is 1 if power is more than 1 it will confuse itself and like this we keep the ratio, the same without wasting power.
            // double denominator = Math.max(Math.abs(forwardpower), Math.max(Math.abs(sidepower), Math.abs(turnpower)));

            // Calculate DC Motor Powers
            // frontRightPower = (forwardpower - sidepower - turnpower) / denominator;
            // frontLeftPower = (forwardpower + sidepower + turnpower) / denominator;
            // backRightPower = (forwardpower + sidepower - turnpower) / denominator;
            // backLeftPower = (forwardpower - sidepower + turnpower) / denominator;

            // Calculate Mecanum Powers
            drivetrain.driveFieldCentric(sidepower, forwardpower, turnpower, angles.thirdAngle);

            // ArmClaw up DPAD_UP
            if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) != true && gamepad2.wasJustPressed(GamepadKeys.DPAD_UP)) {
                clawPower = multiplier;
            }
            // ArmClaw down DPAD_DOWN
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) != true && gamepad2.wasJustPressed(GamepadKeys.DPAD_DOWN)) {
                clawPower = -multiplier;
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) && gamepad2.wasJustPressed(GamepadKeys.DPAD_UP)) {
                capperPower = 1;
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) && gamepad2.wasJustPressed(GamepadKeys.DPAD_DOWN)) {
                capperPower = -1;
            }
            // Stop armClaw DPAD_LEFT
            else if(gamepad2.wasJustPressed(GamepadKeys.DPAD_LEFT)) {
                clawPower = 0;
            }
            // Stop Capper
            else {
                capperPower = 0;
            }
            // Set the accordinate powers to armClaw and capper
            capper.setPower(capperPower);
            claw.setPower(clawPower);


            // INTAKE CODE
            if(gamepad2.wasJustPressed(GamepadKeys.cross)) {
                if(isCollectorActive && collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } else if(gamepad2.wasJustPressed(GamepadKeys.triangle)) {
                if(isCollectorActive && !collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = false;
                }
            }

            // DUCK SPINNER CODE
            if(gamepad1.wasJustPressed(GamepadKeys.square)) {
                duckSpinnersPower = duckSpinnersPower == multiplier-0.25 ? 0 : multiplier-0.25;
            }
            duckSpinners.setPower(duckSpinnersPower);

            if(isCollectorActive) {
                collector.setPower(collectorDirection ? 1 : -multiplier);
            } else collector.setPower(0);

            // Telemetry
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            telemetry.addData("Turn amount: ", calculateAngle360(angles.firstAngle));
            telemetry.addData("frontRight: ", -(forwardpower + sidepower - turnpower));
            telemetry.addData("frontLeft: ", (forwardpower - sidepower + turnpower));
            telemetry.addData("backRight: ", -(forwardpower - sidepower - turnpower));
            telemetry.addData("backLeft: ", (forwardpower + sidepower + turnpower));
            telemetry.addData("Claw :", armClaw.getPower());
            telemetry.addData("Collector: ", collector.getPower());
            telemetry.update();
        }
    }

    private double makePositive(double num) {
        if(num < 0) return -num;
        return num;
    }

    private double calculateAngle360(double num) {
        if(num < 0) return 360+num;
        return num;
    }
}
