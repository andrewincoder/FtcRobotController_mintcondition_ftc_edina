/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * Autonomous file for the goBILDA StarterBot (BIOBUZZ season), MECANUM drive version.
 *
 * Changes from the skid-steer version:
 *  - Four drive motors (left_front_drive, right_front_drive, left_back_drive, right_back_drive)
 *    instead of two.
 *  - drive() sends the same target to all four wheels.
 *  - NEW strafe() moves the robot sideways (positive = right).
 *  - rotate() accounts for the mecanum geometry (uses track width AND wheelbase).
 *  - Completion checks look at all four motors, not just one.
 *
 * The robot starts against the wall, launches, then drives away from the wall.
 */
@Autonomous(name="StarterBotAutonomous", group = "robot")
//@Disabled
public class StarterBotAuto extends OpMode {
    /*
     * Launcher velocity in encoder ticks per second (28 ticks/rev motor encoder).
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Driving constants. DRIVE_SPEED and ROTATE_SPEED are 0-1.
     *
     * TRACK_WIDTH_MM : distance between the centers of the left and right wheels.
     * WHEEL_BASE_MM  : distance between the centers of the front and back wheels.
     *                  >>> MEASURE THIS ON YOUR ROBOT. 300 is only a placeholder. <<<
     * STRAFE_CORRECTION: mecanum wheels lose some efficiency when strafing, so the wheels have
     *                  to turn farther than for a forward move. 1.0 = no correction. Typical
     *                  values are 1.1 - 1.2. Command a 600mm strafe, measure how far the robot
     *                  actually went, and adjust:  new = old * (commanded / actual).
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;
    final double WHEEL_BASE_MM = 300;
    final double STRAFE_CORRECTION = 1.15;

    // Create a variable to set to the intake.
    double intakePower = 0;

    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * TECH TIP: State Machines (see original file). Add more states here, e.g. STRAFE, ROTATE.
     */
    public enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    public AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;

        /*
         * The strings must match the names in the robot configuration on the Driver Hub /
         * Robot Controller. If your config uses different names, change them here.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * Motor directions: the left side is the mirror of the right side, so one side is
         * reversed. If a positive drive() command makes the robot go backwards, swap
         * FORWARD/REVERSE on BOTH sides. If it veers or spins, one side is wrong.
         * Test with a short drive() before trusting strafe() and rotate().
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders on all four drive motors.
        for (DcMotor m : driveMotors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(BRAKE);
        }
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                if (autoTimer.seconds() > 10) {
                    launch(false);
                    intakePower = 0;
                    driveTimer.reset();
                    autonomousState = AutonomousState.DRIVE;
                }
                break;
            case DRIVE:
                if (drive(DRIVE_SPEED, -120, DistanceUnit.MM, 2)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        intake.setPower(intakePower);

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Front pos", "left (%d), right (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.addData("Back pos", "left (%d), right (%d)",
                leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.addData("Front target", "left (%d), right (%d)",
                leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition());
        telemetry.addData("Back target", "left (%d), right (%d)",
                leftBackDrive.getTargetPosition(), rightBackDrive.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    void launch(boolean input) {
        if (input) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (input && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            // Was "intakePower += 0.5", which grows every loop. Motor power clips at 1 anyway,
            // so just set it directly.
            intakePower = 1.0;
        } else {
            windmillServo.setPower(0);
        }
    }

    /* ---------------------------------------------------------------------------------------
     * Drive helpers
     * ------------------------------------------------------------------------------------- */

    private DcMotor[] driveMotors() {
        return new DcMotor[]{leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive};
    }

    /**
     * Sets a target (in ticks) for each wheel, switches to RUN_TO_POSITION and applies power.
     */
    private void setDriveTargets(int lf, int rf, int lb, int rb, double speed) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);

        for (DcMotor m : driveMotors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(speed);
        }
    }

    /**
     * Largest remaining error, in ticks, across all four drive wheels.
     */
    private double maxDriveErrorTicks() {
        double max = 0;
        for (DcMotor m : driveMotors()) {
            max = Math.max(max, Math.abs(m.getTargetPosition() - m.getCurrentPosition()));
        }
        return max;
    }

    /**
     * Drive straight. Positive = forward, negative = backward.
     *
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if all wheels are within tolerance of the target for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, @NonNull DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        int target = (int) (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // All four wheels turn the same way to go straight.
        setDriveTargets(target, target, target, target, speed);

        if (maxDriveErrorTicks() > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Strafe sideways. Positive = right, negative = left.
     *
     * For a strafe right: the front-left and back-right wheels spin forward, while the
     * front-right and back-left wheels spin backward.
     *
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return true once all wheels have been within tolerance for more than holdSeconds.
     */
    boolean strafe(double speed, double distance, @NonNull DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        int target = (int) (distanceUnit.toMm(distance) * TICKS_PER_MM * STRAFE_CORRECTION);

        setDriveTargets(target, -target, -target, target, speed);

        if (maxDriveErrorTicks() > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotate in place. Positive angle follows the original code's convention
     * (left wheels backward, right wheels forward).
     *
     * With mecanum wheels each wheel sits (trackWidth/2) sideways and (wheelBase/2) lengthwise
     * from the robot center, and the distance a wheel must travel for a given rotation is
     * angle (radians) * (trackWidth + wheelBase) / 2. With a wheelbase of 0 this reduces to the
     * original skid-steer formula (angle * trackWidth / 2).
     *
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if all wheels are within tolerance for more than holdSeconds.
     */
    boolean rotate(double speed, double angle, @NonNull AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle) * ((TRACK_WIDTH_MM + WHEEL_BASE_MM) / 2.0);

        int leftTarget = (int) -(targetMm * TICKS_PER_MM);
        int rightTarget = (int) (targetMm * TICKS_PER_MM);

        setDriveTargets(leftTarget, rightTarget, leftTarget, rightTarget, speed);

        if (maxDriveErrorTicks() > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}
