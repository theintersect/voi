package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Howard on 12/5/16.
 * Intake - sweeper and conveyor control
 */

public class VOISweeper {

    private CRServo sweeper1, sweeper2, sweeper3;

    public VOISweeper(CRServo sweeper1, CRServo sweeper2, CRServo sweeper3) {
        this.sweeper1 = sweeper1;
        this.sweeper2 = sweeper2;
        this.sweeper3 = sweeper3;
        sweeper1.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper2.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper3.setDirection(DcMotorSimple.Direction.REVERSE);
        setPower(0);
    }
    public void setPower(double power) {
        sweeper1.setPower(power);
        sweeper2.setPower(power);
        sweeper3.setPower(power);
    }

}
