package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BioBuzz Launch Only Auto", group = "StarterBot")
public class BioBuzzLaunchOnlyAuto extends LinearOpMode {

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Same values as your teleop
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // How long to keep feeding elements into the launcher (seconds)
    public final double LAUNCH_DURATION = 10.0;
    // Safety: give up waiting for spin-up after this many seconds
    public final double SPINUP_TIMEOUT = 4.0;

    @Override
    public void runOpMode() {
        // ---------- INIT ----------
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        launcher.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized - ready to launch");
        telemetry.update();

        waitForStart();

        // ---------- AUTO ----------
        if (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime();

            // 1) Spin up the launcher and wait until it's fast enough
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            timer.reset();
            while (opModeIsActive()
                    && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY
                    && timer.seconds() < SPINUP_TIMEOUT) {
                telemetry.addData("Status", "Spinning up");
                telemetry.addData("Velocity", launcher.getVelocity());
                telemetry.update();
            }

            // 2) Feed elements into the launcher for LAUNCH_DURATION seconds
            timer.reset();
            while (opModeIsActive() && timer.seconds() < LAUNCH_DURATION) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Only feed when the launcher is up to speed (same logic as teleop)
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    windmillServo.setPower(1);
                    intake.setPower(0.5);
                    leftIntakeServo.setPower(0.5);
                    rightIntakeServo.setPower(0.5);
                } else {
                    windmillServo.setPower(0);
                    intake.setPower(0);
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);
                }

                telemetry.addData("Status", "Launching");
                telemetry.addData("Velocity", launcher.getVelocity());
                telemetry.addData("Time left", "%.1f", LAUNCH_DURATION - timer.seconds());
                telemetry.update();
            }

            // 3) Shut everything down
            windmillServo.setPower(0);
            intake.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
            launcher.setVelocity(0);

            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }
}
