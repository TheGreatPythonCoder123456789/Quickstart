package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PoseDebugger", group = "Debug")
public class PoseDebugger extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            Pose p = follower.getPose();

            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }
    }
}