package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Strafe Test")
public class RawStrafeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("motor_lf");
        DcMotor bl = hardwareMap.dcMotor.get("motor_lb");
        DcMotor fr = hardwareMap.dcMotor.get("motor_rf");
        DcMotor br = hardwareMap.dcMotor.get("motor_rb");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double power = 0.5;
        // Standard mecanum strafe left
        fl.setPower(-power);
        fr.setPower( power);
        bl.setPower( power);
        br.setPower(-power);

        sleep(2000);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}