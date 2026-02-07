package org.firstinspires.ftc.teamcode.Chennu.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOpMode30511;

// Add these for Odometry/Pinpoint support


@TeleOp(name = "30511 Bot", group = "SecondBot")
//@Disabled
public class Modif_TeleOpMode30511 extends OpMode {
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

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

        /*
         * Odometry
         */
        //robotOdometry = new Odometry(hardwareMap);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    public void updateAngler(){
        if(gamepad1.dpad_up){
            anglerSystemState = AnglerState.UP;
        } else if (gamepad1.dpad_down) {
            anglerSystemState = AnglerState.DOWN;
        } else{
            anglerSystemState = AnglerState.IDLE;
        }
        switch(anglerSystemState){
            case UP:
                left_turret_angler.setPower(1.0);
                right_turret_angler.setPower(1.0);
                break;
            case DOWN:
                left_turret_angler.setPower(-1.0);
                right_turret_angler.setPower(-1.0);
                break;
            case IDLE:
                left_turret_angler.setPower(0.0);
                right_turret_angler.setPower(0.0);
                break;
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Emergency stop FIRST (works regardless of state)
        if (gamepad1.b) {
            topLauncher.setPower(0);
            bottomLauncher.setPower(0);
            topLauncher.setVelocity(0);
            bottomLauncher.setVelocity(0);

            feeder.setPower(0);
            launchBallState = LaunchState.IDLE;
            intake.setPower(0); intakeState = IntakeState.OFF;
            spinUpTimer.reset();
            FeederTimer.reset();
            return;
        }

        // Manual flywheel spin-up only when not actively launching
        if (launchBallState == LaunchState.IDLE) {
            if (gamepad1.y) {
                topLauncher.setVelocity(launcherTarget);
                bottomLauncher.setVelocity(launcherTarget);
            }
        }

        // Intake toggle
        if (gamepad1.a) {
            if (intakeState == IntakeState.OFF) {
                intakeState = IntakeState.ON;
                intake.setPower(1);
            } else {
                intakeState = IntakeState.OFF;
                intake.setPower(0);
            }
        }

        // Distance toggle
        if (gamepad1.dpad_up) {
            if (launcherDistance == LauncherDistance.CLOSE) {
                launcherDistance = LauncherDistance.FAR;
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
            } else {
                launcherDistance = LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
            }
        }

        boolean manualFeed = gamepad1.left_bumper;                 // Option A: hold to feed
        boolean shotRequested = gamepad1.right_bumper;  // tap to shoot

        launchBall(shotRequested, manualFeed);

        telemetry.addData("LaunchState", launchBallState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("TopVel", topLauncher.getVelocity());
        telemetry.addData("BottomVel", bottomLauncher.getVelocity());
        telemetry.addData("ManualFeed", manualFeedHeld);
        telemetry.addData("TimedFeed", timedFeedActive);
        telemetry.addData("FeederRunning", feederShouldRun);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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





