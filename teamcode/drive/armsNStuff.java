package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.ftccommon.configuration.RobotConfigMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;

public class armsNStuff {


    private HardwareMap hardwareMap;
    DcMotor intake;
    CRServo outtake;
    public armsNStuff(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "outtake");
    }
    public void spikeMark(double power) {
        intake.setPower(power);
    }

    public void backdrop(double power) {
        outtake.setPower(power);


    }

    public void stopPower() {
        intake.setPower(.0);
        outtake.setPower(0.0);
    }
}
