package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private NormalizedColorSensor colorSensor=null;
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

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "coloursensor");
        // Read the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            colors = colorSensor.getNormalizedColors();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);
            telemetry.addData("Power on left intake", leftIntakePower);
            telemetry.addData("Power on right intake", rightIntakePower);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));


            telemetry.update();
        }
    }
}

