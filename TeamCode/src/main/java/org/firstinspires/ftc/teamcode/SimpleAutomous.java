package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class SimpleAutomous extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightmotor;

    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightmotor = hardwareMap.get(DcMotor.class, "right_drive");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            leftMotor.setPower(0.5);
            rightmotor.setPower(0.5);
            sleep(2000);

            leftMotor.setPower(0);
            rightmotor.setPower(0);
            sleep(500);

            leftMotor.setPower(-0.5);
            rightmotor.setPower(-0.5);
            sleep(2000);

            leftMotor.setPower(0);
            rightmotor.setPower(0);
        }

    }





}
