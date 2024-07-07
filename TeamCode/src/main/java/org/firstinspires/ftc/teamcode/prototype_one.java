package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp(name="prototype one", group="brion's opmodes!")
public class prototype_one extends OpMode {

    // hardware
    private DcMotor left_motor;
    private DcMotor right_motor;

    // april tag
    private AprilTagProcessor tag_processor;
    private AprilTagDetection desired_tag = null;

    // buttons
    private Buttons buttons;
    private int tag_absent_frames = 0;

    // toggles
    private boolean tank_mode;
    private boolean BALL_CAPTURED = false; // is the ball in the holder.
    private int TAG_ID = -1; // choose the tag you want to approach or set to -1 for any tag

    // constants
    private final float SCALE = 0.5f;
    private final float DISTANCE_FROM_TAG = 12; // inches
    private final int HIGH_DECIMATION = 3;
    private final int LOW_DECIMATION = 2;
    private final int HIGH_DECIMATION_RANGE = 12;
    private final int FRAMES_BEFORE_LOW_DECIMATION = 4;

    /*
    to turn on the camera stream - click the three dots then camera stream.
     */

    @Override
    public void start(){
        resetRuntime();
        telemetry.addData("status", "initialized");
        telemetry.update();
    }
    //
    // name age hobbies future plans why you joined fgc
    //

    private void setPower(float left, float right){
        left_motor.setPower(left * SCALE + triggerScale(left));
        right_motor.setPower(right * SCALE + triggerScale(right));
    }

    private double triggerScale(double power){ // CONCEPT: if the speed is scaled down and you want to go faster press down right trigger to increase speed.
        double output = gamepad1.right_trigger * SCALE;

        if (power < 0){ // if it's negative you want it to approach -1
            output *= -1;
        } else if (power == 0) { // if it's 0 you don't want to add anything to it
            output = 0;
        }

        return output;
    }

    private void driveSimple(){

        // calculation
        float power_left = -gamepad1.left_stick_y - gamepad1.left_stick_x; // power - turn
        float power_right = gamepad1.left_stick_y + gamepad1.left_stick_x; // power + turn

        // normalization
        if (Math.abs(power_left) > 1 || Math.abs(power_right) > 1){

            float max = Math.max(Math.abs(power_left), Math.abs(power_right));
            power_left /= max;
            power_right /= max;

        }

        setPower(power_left, power_right);

    }

    private void driveTank(){setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);}

    private void driveAuto(){ // drive with apriltags

        ArrayList<AprilTagDetection> detections = tag_processor.getDetections();

        if (detections.isEmpty()){
            tag_absent_frames++;
            if (tag_absent_frames >= FRAMES_BEFORE_LOW_DECIMATION){
                tag_processor.setDecimation(LOW_DECIMATION);
            }
        } else {
            tag_absent_frames = 0;

            if (detections.get(0).ftcPose.z <= HIGH_DECIMATION_RANGE){
                tag_processor.setDecimation(HIGH_DECIMATION);
            }

            for (AprilTagDetection d : detections) {

                if (d.metadata != null) { // if we have  info on this tag

                    if ((TAG_ID < 0) || (d.id == TAG_ID)) { // if we want to go to this tag
                        desired_tag = d;
                        break;
                    }

                }

            }

            if (desired_tag != null){

                // begin moving towards it
                float power = (float) desired_tag.ftcPose.range - DISTANCE_FROM_TAG; // distance between tag and camera (also considered the power)
                float turn = (float) desired_tag.ftcPose.x;

                float power_left = power - turn;
                float power_right = power + turn;

                float max = Math.max(Math.abs(power_left), Math.abs(power_right));
                power_left /= max;
                power_right /= max;

                setPower(power_left, power_right);

            }

        }

    }

    @Override
    public void init() {

        // set up telemetry
        telemetry.setMsTransmissionInterval(50);

        // set up motor
        left_motor = hardwareMap.get(DcMotor.class, "motor1");
        right_motor = hardwareMap.get(DcMotor.class, "motor2");

        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam1");

        // set up buttons
        buttons = new Buttons();

        // april tag
        tag_processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal vision_portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(tag_processor)
                .build();

    }

    @Override
    public void loop() {

        // buttons
        boolean options = buttons.ifPressed(gamepad1.options, true);

        if (options) {
            tank_mode = !tank_mode;
        }

        // drive mode
        if (tank_mode){
            driveTank();
        } else {
            driveSimple();
        }

        // auto drive
        if (gamepad1.a) {
            driveAuto();
        }

        // telemetry
        telemetry.addData("status", "running");
        telemetry.addData("left joystick", String.format(Locale.getDefault(), "(%f, %f)", gamepad1.left_stick_x, -gamepad1.left_stick_y));
        telemetry.addData("right joystick", String.format(Locale.getDefault(), "(%f, %f)", gamepad1.right_stick_x, -gamepad1.right_stick_y));
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("left wheels power", left_motor.getPower());
        telemetry.addData("right wheels power", right_motor.getPower());

        if (desired_tag != null){

            telemetry.addData("\napril tag", "ID %d (%s)", desired_tag.id, desired_tag.metadata.name);
            telemetry.addData("range",  "%.1f inches", desired_tag.ftcPose.range);
            telemetry.addData("bearing","%.0f degrees", desired_tag.ftcPose.bearing);

        }

        telemetry.update();

    }
}

// DEFAULTS FOR APRIL TAG PROCESSOR
// The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

