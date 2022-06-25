package org.firstinspires.ftc.teamcode.Autonomous.New.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;
import org.firstinspires.ftc.teamcode.Configurable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Red Left Storage Unit", group="FTC22Auto_Store")
public class RedAllianceLeftStorageUnit extends RedAllianceLeft {

    @Override
    public void runOpMode() {
        super.runOpMode();
        driveTrain.turn(180, 0.1, 1);
        driveTrain.driveCM(-38, 0.2);
        driveTrain.turn(180, 0.1, 1);
    }
}