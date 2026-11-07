
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name="badAuto", group="StarterBot")
public class easyBiobuzzAuto extends OpMode
{

    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Track width is used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;


    // Create a variable to set to the intake.
    double intakePower = 0;

    /*
     * Here we create a timer which we use in our drive function, and a timer to control the flow
     * of our auto as a whole.
     */
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor leftfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;



    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting
         * autonomousState = AutonomousState.LAUNCH. Later in our code, we will progress through
         * the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
//        autonomousState = BiobuzzStarterbotAuto.AutonomousState.LAUNCH;


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftfrontDrive = hardwareMap.get(DcMotor.class, "leftfrontDrive");
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftbackDrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontDrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
//        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftfrontDrive.setZeroPowerBehavior(BRAKE);
        rightfrontDrive.setZeroPowerBehavior(BRAKE);
        leftbackDrive.setZeroPowerBehavior(BRAKE);
        rightbackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        autoTimer.reset();

            leftfrontDrive.setPower(0.5);
            leftbackDrive.setPower(0.5);
            rightfrontDrive.setPower(0.5);
            rightbackDrive.setPower(0.5);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        leftfrontDrive.setPower(0.0);
            leftbackDrive.setPower(0.0);
            rightfrontDrive.setPower(0.0);
            rightbackDrive.setPower(0.0);
    }
    @Override
    public void loop() {
        // Drive forward for 1.5 seconds at DRIVE_SPEED (0.5), then stop
//        if (autoTimer.seconds() < 1.5) {
//            leftfrontDrive.setPower(1);
//            leftbackDrive.setPower(1);
//            rightfrontDrive.setPower(1);
//            rightbackDrive.setPower(1);
//        } else {
//            leftfrontDrive.setPower(0.0);
//            leftbackDrive.setPower(0.0);
//            rightfrontDrive.setPower(0.0);
//            rightbackDrive.setPower(0.0);
//        }

        telemetry.addData("Auto Timer", autoTimer.seconds());
        telemetry.update();
    }
}
