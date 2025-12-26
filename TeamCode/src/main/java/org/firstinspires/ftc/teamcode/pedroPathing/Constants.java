package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SwyftDrivetrain;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.54)
            .forwardZeroPowerAcceleration(-33)
            .lateralZeroPowerAcceleration(-48.24)
            .headingPIDFCoefficients(new PIDFCoefficients(.44, 0, 0, .025))
            .translationalPIDFCoefficients(new PIDFCoefficients(.025,0,0,.019))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, .6, .15))
            .centripetalScaling(.0004);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.28) //for hardwood
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(false) //Affects DriveTrain.startTeleopDrive() overload.
            .xVelocity(22.9)
            .yVelocity(20.5);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.4375)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .9, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        Follower follower =
        new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new SwyftDrivetrain(hardwareMap, driveConstants))
//                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

        /*
         * Follower has its own globalMaxPower that is initialized to 1.
         * Irritatingly, this cannot be changed using FollowerConstants.
         * It can only be changed by calling setMaxPower() on the built instance
         * of Follower. Set globalMaxPower to be the the same as that in DriveConstants.
         */
        follower.setMaxPower(driveConstants.getMaxPower());
        return follower;
    }
}
