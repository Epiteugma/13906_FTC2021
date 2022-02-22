package org.firstinspires.ftc.teamcode.Drive;

// Navigation and IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Sensors , Motors and Opmode
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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
    public double currentCargoDistance = 0;
    double collectorBoxHeight = 0;
    boolean collectedFreight = false;
    SensorRevTOFDistance cargoDetector = null;

    private String cargoDetection(){
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
        int collectorTicksPerRevolution = 1120;
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // duckSpinners
        boolean duckSpinnersEnabled = false;
        // power factors
        double multiplier = 0.75;
        double globalpowerfactor = 1.0;
        // Arm and positions
        //TODO: Calibrate the ticks needed for each of the 3 levels
        double armTickPerRev = 1120.0;
        double armGearRatio = (double) 125 / 15;
        double armPositionalPower = 0.0;
        double armPower = 0.75;
        int lowPosition = -500;
        int midPosition = -1000;
        int highPosition = -1550;
        int lastClawPosition = arm.getCurrentPosition();

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

            if (globalpowerfactor > 1){
                globalpowerfactor = 1;
            }
            else if(globalpowerfactor < -1){
                globalpowerfactor = 0;
            }

            // Calculate Mecanum Powers for field-centric drive
            drivetrain.driveFieldCentric(sidepower, forwardpower, turnpower, heading);

            // Arm predifined positions
            arm.setRunMode(Motor.RunMode.PositionControl);
            arm.setPositionTolerance(40); // has to be close to the teeth of the small gear
            if (gamepad2.getButton(CROSS)) {
                lastClawPosition = lowPosition;
                armPositionalPower = 0.3;
                if (!arm.atTargetPosition()) {
                    arm.set(armPositionalPower);
                }
            }
            else if (gamepad2.getButton(CIRCLE)) {
                lastClawPosition = midPosition;
                armPositionalPower = 0.25;
                if (!arm.atTargetPosition()) {
                    arm.set(armPositionalPower);
                }
            }
            else if (gamepad2.getButton(TRIANGLE)) {
                lastClawPosition = highPosition;
                armPositionalPower = 0.2;
                if (!arm.atTargetPosition()) {
                    arm.set(armPositionalPower);
                }
            }
            else{

            }

            // Arm up DPAD_UP
            if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_UP)) {
                arm.setRunMode(Motor.RunMode.RawPower);
                arm.set(-0.5);
            }
            // Arm down DPAD_DOWN
            else if(gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && gamepad2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                arm.setRunMode(Motor.RunMode.RawPower);
                arm.set(0.5);
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
                // ARM KEEP POSITION!!!
                lastClawPosition = arm.getCurrentPosition();
                armPositionalPower = 0.1;
                arm.setTargetPosition(lastClawPosition);
                //armPositionalPower = +(+lastClawPosition / 275.0 /10.0);
                if (!arm.atTargetPosition()){
                    arm.set(armPositionalPower);
                }
            }

            // INTAKE CODE
            if(gamepad2.getButton(GamepadKeys.Button.BACK)) {
                if(isCollectorActive && collectorDirection) {
                    isCollectorActive = false;
                }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } 
            else if(gamepad2.getButton(GamepadKeys.Button.START)) {
                if(isCollectorActive && !collectorDirection) {
                    isCollectorActive = false;
                }
                else {
                    isCollectorActive = true;
                    collectorDirection = false;
                }
            }

            // DUCK SPINNER CODE
            if(gamepad1.getButton(SQUARE)) {
                if (!duckSpinnersEnabled) {
                    duckSpinnersPower = 0.35;
                    duckSpinnersEnabled = true;
                }
                else {
                    duckSpinnersEnabled = false;
                    duckSpinnersPower = 0;
                }
            }
            duckSpinners.set(duckSpinnersPower);

            if(isCollectorActive) {
                collector.set(collectorDirection ? +globalpowerfactor + 0.3 : -globalpowerfactor - 0.3);
            } 
            else {
                collector.stopMotor();
            }

            // Telemetry
            detectedCargo = cargoDetection();
            if (!detectedCargo.equals("None") && prevDetectedCargo.equals("None")) {
                // Beta stop the intake when freight is collected and vibrate the drivers' controllers to make them aware
                isCollectorActive = false;
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
