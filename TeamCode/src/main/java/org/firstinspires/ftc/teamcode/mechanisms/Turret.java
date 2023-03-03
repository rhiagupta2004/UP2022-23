package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public  class Turret implements Mechanism {
    private DcMotor turret = null;
    private int turretPos;

    @Override
    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
        turretPos = 0;
        turret.setTargetPosition(turretPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void run(Gamepad gamepad){
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        if(x*x+y*y > 0.1) {
            double angle = Math.atan2(x, y) - Math.PI / 2;
            turretPos = (int) ((angle / Math.PI) * Constants.TURRET_LEFT);
        }
        turretPos += Constants.TURRET_SENSITIVITY * ((gamepad.left_bumper?1:0) - (gamepad.right_bumper?1:0));
        turret.setTargetPosition(turretPos);
    }
    public double getTurretPosition() {
        return turretPos;
    }

    public void setTurretPosition(int pos) {
        turretPos = pos;
        turret.setTargetPosition(turretPos);
    }
}
