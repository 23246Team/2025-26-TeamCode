package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ForwardLeft", group="Robot")
public class forwardLeft extends LinearOpMode {

    private DcMotor frontLeftDrive  = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive  = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double kFront = 1.0;
    static final double kBack  = 1.0;

    @Override
    public void runOpMode() {

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // Forward 3 seconds
        runtime.reset();
        setOmniDrivePowers(FORWARD_SPEED, 0.0, 0.0);
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            telemetry.addData("Path", "Forward: %4.1f s", runtime.seconds());
            telemetry.update();
        }

        // Strafe LEFT 3 seconds (negative lateral)
        runtime.reset();
        setOmniDrivePowers(0.0, -FORWARD_SPEED, 0.0);
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            telemetry.addData("Path", "Strafe Left: %4.1f s", runtime.seconds());
            telemetry.update();
        }

        // Stop
        setOmniDrivePowers(0.0, 0.0, 0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(500);
    }

    private void setOmniDrivePowers(double axial, double lateral, double yaw) {
        double frontLeftPower  = (axial + lateral - yaw) * kFront;
        double frontRightPower = (axial - lateral + yaw) * kFront;
        double backLeftPower   = (axial - lateral - yaw) * kBack;
        double backRightPower  = (axial + lateral + yaw) * kBack;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}

