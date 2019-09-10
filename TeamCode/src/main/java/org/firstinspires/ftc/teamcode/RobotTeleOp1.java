package org.firstinspires.ftc.teamcode;
//Sarang

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name ="RobotTeleOp1", group="Robot")
public class RobotTeleOp1 extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    private Servo scoopServo;
    int positionx = 0;
    int positiony = 1;
    double positionb = .75;
    double positiona = .5;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        //scoopServo = hardwareMap.servo.get("scoopServo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(-gamepad1.left_stick_y);
        //moves left motor according to left joystick (controller1)
        rightMotor.setPower(-gamepad1.right_stick_y);
        //moves right motor according to left joystick (controller1)
        armMotor.setPower(gamepad2.right_stick_y);
        //moves arm according to the left joystick (controller2)
        /*if (gamepad2.x==true) {
            // move to 0 degrees.
            scoopServo.setPosition(positionx);
        } else if (gamepad2.y==true) {
            // move to 180 degrees.
            scoopServo.setPosition(positiony);
        } else if (gamepad2.b==true) {
            // move to 135 degrees
            scoopServo.setPosition(positionb);
        } else if (gamepad2.a==true) {
            // move to 90 degrees
            scoopServo.setPosition(positiona);
        }*/



    }


}

