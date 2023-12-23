package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="hooTelenew", group="Linear Opmode")
//@Disabled
public class hooTelenew extends LinearOpMode {

    boolean armStay = false;

    //declarations and stuff
    private ElapsedTime runtime = new ElapsedTime();


    //gp1 declarations
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    Servo planeLauncherServo = null;

    //gp2 declarations


    // motor speed variables
    double J; double P; double G; double R;

    // joystick position variables
    double X1; double Y1; double X2; double Y2;

    // scalars
    double joyScale = 1.0;
    double motorMax = 1.0; // limit motor power to this value for Andymark~ RUN_USING_ENCODER mode


    @Override
    public void runOpMode() {

        planeLauncherServo = hardwareMap.servo.get("planeLauncherServo");

        leftFront = hardwareMap.dcMotor.get("leftFront"); //FrontLeft
        rightFront = hardwareMap.dcMotor.get("rightFront"); //FrontRight
        leftRear = hardwareMap.dcMotor.get("leftRear"); //BackLeft
        rightRear = hardwareMap.dcMotor.get("rightRear"); //BackRight

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        planeLauncherServo.setDirection(Servo.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // wait for the game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // reset speed variables
            J = 0; P = 0; G = 0; R = 0;

            // get joystick values
            Y1 = gamepad1.right_stick_y * joyScale; // negative inverts so up is positive (correcting for motor mount direction)
            X1 = gamepad1.right_stick_x * joyScale; // frank here, idk if we still need this, look into that when you have a working chassis
            Y2 = gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale;

            // forward/back movement
            J -= Y2;
            P -= Y2;
            G -= Y2;
            R -= Y2;

            // side to side movement
            J += X2;
            P -= X2;
            G -= X2;
            R += X2;

            // rotation
            J += X1;
            P -= X1;
            G += X1;
            R -= X1;

            // Slow Turning
            if (gamepad1.dpad_right){
                J += .4; P -= .4; G += .4; R -= .4;
            }
            if (gamepad1.dpad_left){
                J -= .4; P += .4; G -= .4; R += .4;
            }
            if (gamepad1.dpad_up){
                J += .4; P += .4; G += .4; R += .4;
            }
            if (gamepad1.dpad_down){
                J -= .4; P -= .4; G -= .4; R -= .4;
            }

            if (gamepad1.y) {
                planeLauncherServo.setPosition(0.7);  // Adjust the position value as needed
            } else {
                planeLauncherServo.setPosition(0.3);  // Set to 0 when not activated
            }

            //motor power limiter
            J = Math.max(-motorMax, Math.min(J, motorMax)); //FrontLeft
            P = Math.max(-motorMax, Math.min(P, motorMax)); //FrontRight
            G = Math.max(-motorMax, Math.min(G, motorMax)); //BackLeft
            R = Math.max(-motorMax, Math.min(R, motorMax)); //BackRight

            //set power to wheels
            leftFront.setPower(J);
            rightFront.setPower(P);
            leftRear.setPower(G);
            rightRear.setPower(R);



            // send some useful parameters to the driver station
            // (outputting the current wheel speed)
            telemetry.addData("LF", "%.3f", J);
            telemetry.addData("RF", "%.3f", P);
            telemetry.addData("LR", "%.3f", G);
            telemetry.addData("RR", "%.3f", R);

            telemetry.addData("Launcher Position", planeLauncherServo.getPosition());

            telemetry.update();


            //gamepad 2 controls (gunner)









        }



    }
}


//Expansion Hub: Arm Up Down --> 0
//Arm Extend Retract --> 2
//Arm Servo --> 0