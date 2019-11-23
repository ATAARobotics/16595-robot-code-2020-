package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="BlueImport: Auto Drive By Encoder", group="")
public class Autocode_BlueImport  extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private ColorSensor colorSensor=null;
    private double maxPower = 0.1;
    private Servo servoArm = null;
    private double servoArmDown =0.5;
    private double servoArmUp =-0.5;
    private DcMotorSimple leftIntake = null;
    private DcMotorSimple rightIntake = null;
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;
    private boolean intake = false;
    private int COUNTS_PER_INCH=764;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;
        colorSensor = hardwareMap.get(ColorSensor.class, "coloursensor");

        // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftbackDrive  = hardwareMap.get(DcMotor.class, "leftbackdrive");
    rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
    leftfrontDrive  = hardwareMap.get(DcMotor.class, "leftfrontdrive");
    rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
    servoArm = hardwareMap.get(Servo.class, "servoarm");
    leftIntake = hardwareMap.get(DcMotorSimple.class, "leftIntake");
    rightIntake = hardwareMap.get(DcMotorSimple.class, "rightIntake");

    colorSensor = hardwareMap.get(ColorSensor.class, "coloursensor");
    // Read the sensor
    //NormalizedRGBA colors = colorSensor.getNormalizedColors();

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
    leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
    rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
    servoArm.setPosition(servoArmDown);
    servoArm.setDirection(Servo.Direction.REVERSE) ;
    leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        mechanumEncoderDrive(maxPower,  48,  5.0);

        encoderDrive(maxPower,  15,  15, 5.0, true);
        if (sampleColor(Color.RED) && (sampleColor(Color.BLUE)) && (sampleColor(Color.GREEN))){
        servoArm .setPosition(servoArmDown);
            while (servoArm.getPosition() > servoArmDown) {}
        }

        encoderDrive(maxPower,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(maxPower,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(maxPower, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Status", "Run Time: " + runtime.toString());
 //       telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", frontleftPower, frontrightPower);
 //       telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", backleftPower, backrightPower);
        telemetry.addData("ServoArm Position", "left (%.2f)", servoArm.getPosition());
        telemetry.addData("Power on left intake", leftIntakePower);
        telemetry.addData("Power on right intake", rightIntakePower);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void mechanumEncoderDrive(double speed, double rightInches,double timeoutS) {
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget = rightfrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            //rightfrontDrive.setTargetPosition(newRightTarget);
//            leftfrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
//            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setMotor(rightfrontDrive,newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            leftfrontDrive.setPower(Math.abs(speed));
            rightfrontDrive.setPower(Math.abs(speed));
            leftbackDrive.setPower(-Math.abs(speed));
            rightbackDrive.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rightfrontDrive.isBusy())) {

                // Display it for the driver.
                displayDriver(newRightTarget);
            }

            DcMotor[] all = {rightfrontDrive, leftfrontDrive, rightbackDrive, leftbackDrive};
            resetMotors(all, rightfrontDrive);
            // Stop all motion;
//            leftfrontDrive.setPower(0);
//            rightfrontDrive.setPower(0);
//            leftbackDrive.setPower(0);
//            rightbackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
//            rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches,double timeoutS) {
        encoderDrive(speed,leftInches,rightInches,timeoutS,false);
    }
        /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,double leftInches, double rightInches,  double timeoutS, boolean interrupt) {
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget = rightfrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            rightfrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
//            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            setMotor(rightfrontDrive, newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            leftfrontDrive.setPower(Math.abs(speed)* (leftInches > 0?1:-1));
            rightfrontDrive.setPower(Math.abs(speed));
            leftbackDrive.setPower(Math.abs(speed)* (leftInches > 0?1:-1));
            rightbackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rightfrontDrive.isBusy()) ) {

                if(interrupt && (
                        sampleColor(Color.RED)  &&
                        sampleColor(Color.BLUE) &&
                        sampleColor(Color.GREEN) )){
                    break;
                }

                // Display it for the driver.
                displayDriver(newRightTarget);


            }
            DcMotor[] all = {rightfrontDrive, leftfrontDrive, rightbackDrive, leftbackDrive};
            resetMotors(all, rightfrontDrive);
            // Stop all motion;
//            leftfrontDrive.setPower(0);
//            rightfrontDrive.setPower(0);
//            leftbackDrive.setPower(0);
//            rightbackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
//            rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // Color change
    private boolean sampleColor(int color) {
        colorSensor.enableLed(true);
        if(colorSensor.red() <= 8 && colorSensor.red() >= 2) {
            colorSensor.enableLed(false);
            return true;
        }
        else if(colorSensor.blue() <= 3 && colorSensor.blue() >= 2) {
            colorSensor.enableLed(false);
            return true;

        }
        else  if(colorSensor.red() <= 6 && colorSensor.red() >= 3) {
            colorSensor.enableLed(false);
            return true;

        }
        colorSensor.enableLed(false);
        return false;
    }

    public void displayDriver(int target){
        telemetry.addData("Path1",  "Running to %7d",  target);
        telemetry.addData("Front Right",  (double)rightfrontDrive.getCurrentPosition());
        telemetry.addData("Front Left",  (double)leftfrontDrive.getCurrentPosition());
        telemetry.addData("Back Right",  (double)rightbackDrive.getCurrentPosition());
        telemetry.addData("Back Left",  (double)leftbackDrive.getCurrentPosition());
        telemetry.addData("Target Position", (double)rightfrontDrive.getTargetPosition());
        telemetry.update();
    }

    public void setMotor(DcMotor current, int target){
        current.setTargetPosition(target);
        current.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void resetMotors(DcMotor[] all, DcMotor current){
        for(int i=0;i<all.length;i++) {
            all[i].setPower(0);
        }
        current.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
