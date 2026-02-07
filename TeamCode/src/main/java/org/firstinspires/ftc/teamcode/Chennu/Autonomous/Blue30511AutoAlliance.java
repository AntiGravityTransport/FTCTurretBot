package org.firstinspires.ftc.teamcode.Chennu.Autonomous;
/*
Copyright 2026 FIRST Tech Challenge Team 30511

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name = "30511 Blue Alliance- Auto", group = "35011 Auto")
public class Blue30511AutoAlliance extends LinearOpMode {


    final double FEED_TIME_SECONDS = 0.50; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx topLauncher = null;
    private DcMotorEx bottomLauncher = null;
    private DcMotor intake = null;
    private DcMotor feeder = null;
    //private Odometry robotOdometry;
    private CRServo right_rotator = null;
    private CRServo left_rotator = null;
    private CRServo right_turret_angler = null;
    private CRServo left_turret_angler = null;

    ElapsedTime spinUpTimer = new ElapsedTime();
    final double SPINUP_TIMEOUT = 2.0;

    ElapsedTime FeederTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    // === Encoder constants (YOU MUST TUNE THESE FOR YOUR ROBOT) ===
    // Common goBILDA 312RPM motor encoder: ~537.7 ticks/rev (check your motor!)
    private static final double TICKS_PER_REV = 537.7;

    // Wheel diameter in inches (common: 96mm mecanum ≈ 3.78 in, or 4.0 in wheels)
    private static final double WHEEL_DIAMETER_IN = 4.0;

    // Gear reduction from motor to wheel (1.0 if direct drive)
    private static final double GEAR_REDUCTION = 1.0;

    // Track width (distance between left and right wheels) in inches
    // Measure your robot center-to-center left-to-right wheel contact points.
    private static final double TRACK_WIDTH_IN = 15.0;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_IN);

    boolean feederShouldRun = false;
    boolean timedFeedActive = false;
    boolean manualFeedHeld = false;


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchBallState;

    // Place this with your other private variables
    private enum AnglerState {
        IDLE,
        UP,
        DOWN;
    }

    private AnglerState anglerSystemState = AnglerState.IDLE;

    private enum IntakeState {
        ON,
        OFF;
    }

    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;



    @Override
    public void runOpMode() {

        launchBallState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        topLauncher = hardwareMap.get(DcMotorEx.class, "top_launcher");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "bottom_launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //right_rotator = hardwareMap.get(CRServo.class, "right_rotator");
        //left_rotator = hardwareMap.get(CRServo.class, "left_rotator");
        //right_turret_angler = hardwareMap.get(CRServo.class, "right_turret_angler");
        //left_turret_angler = hardwareMap.get(CRServo.class, "left_turret_angler");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        //bottomLauncher.setDirection(DcMotorEx.Direction.REVERSE);

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //right_rotator.setDirection(CRServo.Direction.REVERSE);
        //right_turret_angler.setDirection(CRServo.Direction.REVERSE);

        topLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        topLauncher.setZeroPowerBehavior(BRAKE);
        bottomLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        feeder.setZeroPowerBehavior(BRAKE);


        topLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        bottomLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        //leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(!opModeIsActive()) return;

        // =========================================================
        // 1) Drive forward 12 inches (1 ft)
        // =========================================================
        driveForwardInches(2.0, 0.5, 400);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllDrivePower(double p) {
        leftFrontDrive.setPower(p);
        rightFrontDrive.setPower(p);
        leftBackDrive.setPower(p);
        rightBackDrive.setPower(p);
    }

    private void runToPosition(int lf, int rf, int lb, int rb, double speed, long timeoutMs) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        timer.reset();
        while (opModeIsActive()
                && timer.milliseconds() < timeoutMs
                && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy()
                || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            idle();
        }

        setAllDrivePower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveForwardInches(double inches, double speed, long timeoutMs) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);

        int lf = leftFrontDrive.getCurrentPosition() + ticks;
        int rf = rightFrontDrive.getCurrentPosition() + ticks;
        int lb = leftBackDrive.getCurrentPosition() + ticks;
        int rb = rightBackDrive.getCurrentPosition() + ticks;

        runToPosition(lf, rf, lb, rb, speed, timeoutMs);
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void launchBall(boolean shotRequested, boolean manualFeed) {

        manualFeedHeld = manualFeed;

        switch (launchBallState) {
            case IDLE:
                if (shotRequested) {
                    spinUpTimer.reset();
                    launchBallState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                topLauncher.setVelocity(launcherTarget);
                bottomLauncher.setVelocity(launcherTarget);
                if (topLauncher.getVelocity() > launcherMin &&
                        bottomLauncher.getVelocity() > launcherMin) {
                    launchBallState = LaunchState.LAUNCH;
                } else if (spinUpTimer.seconds() > SPINUP_TIMEOUT) {
                    launchBallState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                FeederTimer.reset();
                launchBallState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (FeederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchBallState = LaunchState.IDLE;
                }
                break;
        }

        timedFeedActive =
                (launchBallState == LaunchState.LAUNCHING) &&
                        (FeederTimer.seconds() <= FEED_TIME_SECONDS);

        feederShouldRun = manualFeedHeld || timedFeedActive;

        feeder.setPower(feederShouldRun ? FULL_SPEED : STOP_SPEED);
    }
}

