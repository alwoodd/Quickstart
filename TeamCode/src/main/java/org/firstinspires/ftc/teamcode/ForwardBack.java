package org.firstinspires.ftc.teamcode;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class ForwardBack extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose endPose = new Pose(56, 56, Math.toRadians(90));

    private Path goForward;
    private Path goBackward;

    private boolean isForward = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        goForward = new Path(new BezierLine(startPose, endPose));
        //goForward.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        //goForward.setTangentHeadingInterpolation();
        goForward.setConstantHeadingInterpolation(endPose.getHeading());

        goBackward = new Path(new BezierLine(endPose, startPose));
        goBackward.setConstantHeadingInterpolation(startPose.getHeading());

        follower.update();
        showTel();

        follower.setMaxPower(.5);
    }

    @Override
    public void loop() {
        showTel();
        follower.update();

        Path currentPath;
        if (!follower.isBusy()) {
            if (isForward) {
                isForward = false;
                currentPath = goForward;
            }
            else {
                isForward = true;
                currentPath = goBackward;
            }
            follower.followPath(currentPath, true);
        }
    }

    private void showTel() {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("IMU Heading", ((PinpointLocalizer)follower.getPoseTracker().getLocalizer()).getPinpoint().getHeading(AngleUnit.DEGREES));
/*
        if (follower.isBusy()) {
            telemetry.addData("Heading target", Math.toDegrees(follower.getCurrentPath().getHeadingGoal(follower.getClosestPose())));
            telemetry.addLine("Drive Vector");
            telemetry.addData("x", follower.getDriveVector().getXComponent());
            telemetry.addData("y", follower.getDriveVector().getYComponent());
        }
*/
        telemetry.update();
//        sleep(1500);
    }
/*
    public void runOpMode() {

        if (!follower.isBusy()) {
            if (isForward) {
                isForward = false;
                follower.followPath(backward);
            }
            else {
                isForward = true;
                follower.followPath(forward);
            }
        }
    }
*/
    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
