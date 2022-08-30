package org.firstinspires.ftc.teamcode.ftc2022.autonomous.opmodes.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ftc2022.Configurable;

@Autonomous(name = "Blue Right Warehouse", group = "FTC22Auto_Ware")
@Disabled
public class RightWarehouse extends Right {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        driveTrain.driveCM(-10, 0.3);
        driveTrain.turn(85, 0.1, 1);
        driveTrain.driveCM(100, 0.4);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(85, 0.1, 1);
        driveTrain.driveCM(280, 0.4);
    }
}
