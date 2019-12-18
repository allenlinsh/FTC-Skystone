package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    // Declare hardware variables
    public BNO055IMU imu;
    public DcMotor leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor;
    public DcMotor armMotor;
    public DcMotor gripMotor;
    public Servo leftServo, rightServo;
    public Servo leftSkystoneServo, rightSkystoneServo;
    public ColorSensor leftColorSensor, rightColorSensor;
    public DigitalChannel topLimit, bottomLimit;
    public WebcamName LogitechC310;

    // Declare general variables
    public ElapsedTime runtime;
    private boolean noStart = true;

    // Declare movement variables
    private static final int ticksPerRev = 480;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double correction;
    private double maxPower = round((100.0/127.0), 2);
    private double minPower = 0.25;
    private double maxGripPower = 0.5;
    private double minTurnPower = 0.35;
    private double slowGain = 0.1;
    private double driveAxial, driveLateral, driveYaw;
    private double lateralFactor = 1.40;
    private double armPower, gripPower;
    private double leftBackPower, rightBackPower, leftFrontPower, rightFrontPower;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize hardware
        getHardwareMap();
        initCheck();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            noStart = !(this.gamepad1.start || this.gamepad2.start);

            driveAxial      = 0;
            driveLateral    = 0;
            driveYaw        = 0;
            armPower        = 0;
            gripPower       = 0;

            // Hook on
            if (this.gamepad1.right_bumper) {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }
            // Hook off
            else if (this.gamepad1.left_bumper) {
                leftServo.setPosition(0.1);
                rightServo.setPosition(0.9);
            }

            // Grip hold
            if (this.gamepad2.right_trigger > 0) gripPower = this.gamepad2.right_trigger * maxGripPower;
            // Grip release
            else if (this.gamepad2.left_trigger > 0) gripPower = -this.gamepad2.right_trigger * maxGripPower;

            // Left skystone grabber
            if (noStart && this.gamepad2.x) {
                // Grabber on
                if (leftSkystoneServo.getPosition() < 0.98) leftSkystoneServo.setPosition(0.98);
                // Grabber off
                else if (leftSkystoneServo.getPosition() > 0.52) leftSkystoneServo.setPosition(0.52);
            }
            // Right skystone grabber
            else if (noStart && this.gamepad2.b) {
                // Grabber on
                if (rightSkystoneServo.getPosition() > 0.52) rightSkystoneServo.setPosition(0.52);
                // Grabber off
                else if (rightSkystoneServo.getPosition() < 0.98) rightSkystoneServo.setPosition(0.98);
            }

            if (this.gamepad1.left_stick_y == 0 && this.gamepad1.left_stick_x == 0 && this.gamepad1.right_stick_x == 0) {
                // Slow left
                if (gamepad1.dpad_left) driveLateral = -(minPower + slowGain) * lateralFactor;
                // Slow right
                if (gamepad1.dpad_right) driveLateral = (minPower + slowGain) * lateralFactor;
                // Slow forward
                if (gamepad1.dpad_up) driveAxial = -(minPower + slowGain);
                // Slow backward
                if (gamepad1.dpad_down) driveAxial = (minPower + slowGain);
                // Slow counter-clockwise
                if (noStart && gamepad1.x) driveYaw = -(minTurnPower + slowGain);
                // Slow clockwise
                if (noStart && gamepad1.b) driveYaw = (minTurnPower + slowGain);
            }
            else {
                // set axial movement to logarithmic values and set a dead zone
                driveAxial = this.gamepad1.left_stick_y;
                if (Math.abs(driveAxial) < Math.sqrt(0.1)) driveAxial = 0;
                else {
                    driveAxial = driveAxial * maxPower;
                    driveAxial = Math.signum(driveAxial) * driveAxial * driveAxial;
                }
                // set lateral movement to logarithmic values and set a dead zone
                driveLateral = this.gamepad1.left_stick_x;
                if (Math.abs(driveLateral) < Math.sqrt(0.1)) driveLateral = 0;
                else {
                    driveLateral = driveLateral * maxPower;
                    driveLateral = Math.signum(driveLateral) * driveLateral * driveLateral;
                    driveLateral = driveLateral * lateralFactor;
                }
                // set yaw movement to logarithmic values and set a dead zone
                driveYaw = this.gamepad1.right_stick_x;
                if (Math.abs(driveYaw) < Math.sqrt(0.1)) driveYaw = 0;
                else {
                    driveYaw = driveYaw * maxPower;
                    driveYaw = Math.signum(driveYaw) * driveYaw * driveYaw;
                }
            }

            leftBackPower = round((-driveLateral - driveAxial + driveYaw), 2);
            rightBackPower = round((driveLateral - driveAxial - driveYaw), 2);
            leftFrontPower = round((driveLateral - driveAxial + driveYaw), 2);
            rightFrontPower = round((-driveLateral - driveAxial - driveYaw), 2);
            if (this.gamepad1.left_stick_y != 0 || this.gamepad1.left_stick_x != 0 || this.gamepad1.right_stick_x != 0 || this. gamepad1.dpad_up || this. gamepad1.dpad_down || this. gamepad1.dpad_left || this. gamepad1.dpad_right || this. gamepad1.x || this. gamepad1.b) {
                run(leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);
            }
            else {
                stopMotor();
            }

            // Arm stops if top limit is on and arm is moving backward
            // or if bottom limit is on and arm is moving forward
            armPower = this.gamepad2.left_stick_y;
            if ((armPower > 0 && topPressed()) || (armPower < 0 && bottomPressed())) armMotor.setPower(0);
            else armMotor.setPower(armPower);

            gripMotor.setPower(gripPower);

            telemetry.addData("4 leftBackPower", leftBackPower);
            telemetry.addData("5 rightBackPower", rightBackPower);
            telemetry.addData("6 leftFrontPower", leftFrontPower);
            telemetry.addData("7 rightFrontPower", rightBackPower);
            telemetry.addData("8 gripPower", gripPower);
            telemetry.addData("9 armPower", armPower);
            telemetry.update();
        }
        resetMotor();
    }
    // General functions
    public void print(String caption, Object message) {
        telemetry.addData(caption, message);
    }
    public double round(double val, int roundTo) {
        return Double.valueOf(String.format("%." + roundTo + "f", val));
    }

    // Init functions
    public void getHardwareMap() {
        imu                 = hardwareMap.get(BNO055IMU.class, "imu");
        leftBackMotor       = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor      = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor      = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor     = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        armMotor            = hardwareMap.get(DcMotor.class, "armMotor");
        gripMotor           = hardwareMap.get(DcMotor.class, "gripMotor");
        leftServo           = hardwareMap.get(Servo.class, "leftServo");
        rightServo          = hardwareMap.get(Servo.class, "rightServo");
        leftSkystoneServo   = hardwareMap.get(Servo.class, "leftSkystoneServo");
        rightSkystoneServo  = hardwareMap.get(Servo.class, "rightSkystoneServo");
        leftColorSensor     = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor    = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        topLimit            = hardwareMap.get(DigitalChannel.class, "topLimit");
        bottomLimit         = hardwareMap.get(DigitalChannel.class, "bottomLimit");
        LogitechC310        = hardwareMap.get(WebcamName.class, "Logitech C310");
    }
    public void initMotor() {
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gripMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean checkMotor() {
        if (gripMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) return true;
        else return false;
    }
    public void initServo() {
        leftServo.setPosition(0.10);
        rightServo.setPosition(0.90);
        leftSkystoneServo.setPosition(0.52);
        rightSkystoneServo.setPosition(0.98);
    }
    public boolean checkServo() {
        if (round(leftServo.getPosition(), 2) == 0.10
                && round(rightServo.getPosition(), 2) == 0.90
                && round(leftSkystoneServo.getPosition(), 2) == 0.52
                && round(rightSkystoneServo.getPosition(), 2) == 0.98) {
            return true;
        }
        else return false;
    }
    public void initIMU() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
    }
    public boolean checkIMU() {
        if (imu.isGyroCalibrated()) return true;
        else return false;
    }
    public void initCheck() {
        while (!isStopRequested() && !(checkMotor() && checkServo() && checkIMU())) {
            // Initialize motor
            if(!checkMotor()) {
                print("Motor","Initializing");
                telemetry.update();
                initMotor();
            }
            else if(checkMotor()) {
                print("Motor","Initialized");
                telemetry.update();
                // Initialize servo
                if(!checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initializing");
                    telemetry.update();
                    initServo();
                    while (!isStopRequested() && !checkServo()) {
                        idle();
                        sleep(50);
                    }
                }
                else if(checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initialized");
                    telemetry.update();
                    // Initialize imu
                    if(!checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initializing...");
                        telemetry.update();
                        initIMU();
                        while (!isStopRequested() && !checkIMU()) {
                            idle();
                            sleep(50);
                        }
                    }
                    else if(checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initialized");
                        telemetry.update();
                    }
                }
            }
        }
    }

    // Movement functions
    public void run(double leftBackPower, double rightBackPower, double leftFrontPower, double rightFrontPower) {
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
    }
    public void stopMotor() {
        run(0, 0, 0, 0);
    }
    public void resetMotor() {
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        armMotor.setPower(0);
        gripMotor.setPower(0);
    }

    public void pause() {sleep(150);}
    public boolean topPressed() {
        return !topLimit.getState();
    }
    public boolean bottomPressed() {
        return !bottomLimit.getState();
    }
}
