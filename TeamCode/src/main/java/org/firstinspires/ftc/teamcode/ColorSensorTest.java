package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ColorTest", group="TeleOp")
public class ColorSensorTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private ColorSensor colorSensor=null;
    private double maxPower = 0.25;
    private Servo servoArm = null;
    private double servoArmDown =0.5;
    private double servoArmUp =-0.5;
    private DcMotorSimple leftIntake = null;
    private DcMotorSimple rightIntake = null;
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;
    private boolean intake = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        colorSensor = hardwareMap.get(ColorSensor.class, "coloursensor");
        // Read the sensor

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Power on left intake", leftIntakePower);
            telemetry.addData("Power on right intake", rightIntakePower);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", colorSensor.alpha())
                    .addData("r", "%02x", colorSensor.red())
                    .addData("g", "%02x", colorSensor.green())
                    .addData("b", "%02x", colorSensor.blue());


            telemetry.update();
        }
    }
}

