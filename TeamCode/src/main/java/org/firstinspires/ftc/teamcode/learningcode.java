package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
@Disabled
public class learningcode extends OpMode {
    @Override
    public void init(){



    }

    @Override
    public void loop(){

        int TeamNumber = 24183;
        double speed = gamepad1.left_stick_y;
        double trigersum = gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData("Hellow", "World");
        telemetry.addData("TeamNumber",TeamNumber);
        telemetry.addData("speed", speed);
        telemetry.addData("left x", gamepad1.left_stick_x);
        telemetry.addData("left y", gamepad1.left_stick_y);
        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", gamepad1.right_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("left triger", gamepad1.left_trigger);
        telemetry.addData("right triger", gamepad1.right_trigger);
        telemetry.addData("triger sum", trigersum);


    }



}
