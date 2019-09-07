package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GearLock", group="Robot")
//@Disabled
public class GearLock extends OpMode {
    /************************************************
     * Motor Controllers
     */
    private DcMotorController dc_drive_controller;
    /*public DcMotor  leftMotor   = hardwareMap.get(DcMotor.class, "leftMotor");
    public DcMotor  rightMotor  = hardwareMap.get(DcMotor.class, "rightMotor");
    public DcMotor  armMotor     = hardwareMap.get(DcMotor.class, "armMotor");
    public Servo gearLock    = hardwareMap.get(Servo.class, "gearLock");*/
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  armMotor     = null;
    public Servo gearLock    = null;
    double          clawPosition    = 0;
    static final double    CLAW_SPEED      = 0.01 ; // sets rate to move servo
    static final double CLAW_MIN_RANGE  = 0.20;
    static final double CLAW_MAX_RANGE  = 0.7;
    static final double CLAW_HOME = 0.2;
    /*********************************************
     * Motors Used
     */

    @Override
    public void init() {
        // dc_drive_controller = hardwareMap.dcMotorController.get("drive_controller");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        gearLock  = hardwareMap.get(Servo.class, "gearLock");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);
        armMotor.setPower(gamepad2.right_stick_y);
        gearLock.scaleRange(CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        if (gamepad2.x)
            clawPosition += CLAW_SPEED;
        else if (gamepad2.b)
            clawPosition -= CLAW_SPEED;
        gearLock.setPosition(clawPosition);

        //to get the current position of the servo:
        //gearLock.getController().getServoPosition(Servo.getChannel()).
        //gearLock = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        //gearLock.scaleRange(CLAW_MIN_RANGE, CLAW_MAX_RANGE);
        //gearLock.setPosition(clawPosition);
    }

    /*
    This added code will check to see if any of the colored buttons on the F310 gamepad are pressed.
    If the Y button is pressed, it will move the servo to the 0-degree position. If either the X button
    or B button is pressed, it will move the servo to the 90-degree position. If the A button is pressed,
    it will move the servo to the 180-degree position. The op mode will also send telemetry data on the
    servo position to the Driver Station.
     */
    public void tryThisServoCodeFromFTC(){
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;

        //while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            //leftMotor.setPower(tgtPower);
            // check to see if we need to move the servo.
            if(gamepad1.y) {
                // move to 0 degrees.
                gearLock.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                gearLock.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                gearLock.setPosition(1);
            }
            telemetry.addData("Servo Position", gearLock.getPosition());
            //telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", leftMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        //}
    }

}