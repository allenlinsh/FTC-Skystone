package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class MainTeleOpCorrection extends LinearOpMode {
    // Declare hardware variables
    public BNO055IMU imu;
    public DcMotor leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor;
    public DcMotor armMotor, gripMotor;
    public Servo leftServo, rightServo;
    public DigitalChannel topLimit, bottomLimit;
    public WebcamName LogitechC310;
    public RevBlinkinLedDriver led;

    // Declare general variables
    public ElapsedTime runtime, gametime;
    public boolean initReady = false;
    private boolean noStart = true;
    public RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public RevBlinkinLedDriver.BlinkinPattern blinkPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

    // Declare movement variables
    private static final int ticksPerRev = 480;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double correction;
    private double maxPower = round((100.0/127.0), 2);
    private double drivePower = round((40.0/127.0), 2);
    private double minPower = 0.25;
    private double maxGripPower = 0.5;
    private double minTurnPower = 0.35;
    private double slowGain = 0.1;
    private double driveAxial, driveLateral, driveYaw;
    private double lateralFactor = 1.40;
    private double armPower, gripPower;
    private double leftBackPower, rightBackPower, leftFrontPower, rightFrontPower;
    private int servoTimePer90Deg = 850;
    public PIDController pidCorrection = new PIDController(.05, 0, 0);

    // Declare shared preference variables
    public SharedPreferences preferences;
    public String teamColor;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        gametime = new ElapsedTime();

        // Initialize hardware
        getHardwareMap();
        getPreferences();
        initCheck();

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

        waitForStart();

        gametime.reset();
        gametime.startTime();
        resetAngle();

        while (opModeIsActive()) {
            noStart = !(gamepad1.start || gamepad2.start);

            driveAxial      = 0;
            driveLateral    = 0;
            driveYaw        = 0;
            armPower        = 0;
            gripPower       = 0;

            //
            // ================================= SERVO CONTROL =====================================
            //
            if (gamepad1.right_bumper) {
                // Hook on
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            } else if (gamepad1.left_bumper) {
                // Hook off
                leftServo.setPosition(0);
                rightServo.setPosition(1);
            }

            //
            // ================================= MOTOR CONTROL =====================================
            //
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0) {
                // Slow forward
                if (gamepad1.dpad_up) driveAxial = -(minPower + slowGain);
                // Slow backward
                if (gamepad1.dpad_down) driveAxial = (minPower + slowGain);
                // Slow left
                if (gamepad1.dpad_left) driveLateral = -(minPower + slowGain) * lateralFactor;
                // Slow right
                if (gamepad1.dpad_right) driveLateral = (minPower + slowGain) * lateralFactor;
                // Slow counter-clockwise
                if (noStart && gamepad1.x) driveYaw = -(minTurnPower + slowGain);
                // Slow clockwise
                if (noStart && gamepad1.b) driveYaw = (minTurnPower + slowGain);
            } else {
                // set axial movement to logarithmic values and set a dead zone
                driveAxial = (Math.abs(gamepad1.left_stick_y) < Math.sqrt(0.1))
                        ? 0
                        : Math.signum(gamepad1.left_stick_y) * Math.pow((gamepad1.left_stick_y * maxPower), 2);
                // set lateral movement to logarithmic values and set a dead zone
                driveLateral = (Math.abs(gamepad1.left_stick_x) < Math.sqrt(0.1))
                        ? 0
                        : Math.signum(gamepad1.left_stick_x) * Math.pow((gamepad1.left_stick_x * maxPower), 2) * lateralFactor;
                // set yaw movement to logarithmic values and set a dead zone
                driveYaw = (Math.abs(gamepad1.right_stick_x) < Math.sqrt(0.1))
                        ? 0
                        : Math.signum(gamepad1.right_stick_x) * Math.pow((gamepad1.right_stick_x * 1.0), 2);
            }
            leftBackPower = round((-driveLateral - driveAxial + driveYaw), 2);
            rightBackPower = round((driveLateral - driveAxial - driveYaw), 2);
            leftFrontPower = round((driveLateral - driveAxial + driveYaw), 2);
            rightFrontPower = round((-driveLateral - driveAxial - driveYaw), 2);

            // Arm stops if top or bottom limit is on
            armPower = ((gamepad2.left_stick_y > 0 && topPressed()) || (gamepad2.left_stick_y < 0 && bottomPressed()))
                    ? 0
                    : gamepad2.left_stick_y;

            if (gamepad2.right_trigger > 0) {
                // Grip hold
                gripPower = gamepad2.right_trigger * maxGripPower;
            } else if (gamepad2.left_trigger > 0) {
                // Grip release
                gripPower = -gamepad2.left_trigger * maxGripPower;
            }
            //
            // ================================= POWER CONTROL =====================================
            //
            if (driveYaw != 0) {
                resetAngle();
            } else {
                correction = pidCorrection.performPID(getAngle());
            }
            if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0
                    || gamepad1.right_stick_x != 0 || gamepad1.dpad_up || gamepad1.dpad_down
                    || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.x || gamepad1.b) {
                run(leftBackPower-correction, rightBackPower+correction, leftFrontPower-correction, rightFrontPower+correction);
            } else {
                stopMotor();
            }
            if (gamepad2.left_stick_y != 0 || gamepad2.right_trigger != 0
                    || gamepad2.left_trigger != 0) {
                armMotor.setPower(armPower);
                gripMotor.setPower(gripPower);
            } else {
                armMotor.setPower(0);
                gripMotor.setPower(0);
            }

            //
            // ================================ FEEDBACK CONTROL ===================================
            //
            blinkLED(5000);
            blinkLED(70000);
            blinkLED(90000);

            print("Status", "Running");
            print("Alliance", teamColor);
            print("time", gametime);
            print("Left Back Power", leftBackPower);
            print("Right Back Power", rightBackPower);
            print("Left Front Power", leftFrontPower);
            print("Right Front Power", rightBackPower);
            print("Arm Power", armPower);
            print("Grip Power", gripPower);
            update();
        }
        stopAllMotors();
    }
    // General functions
    public void print(String caption, Object message) {
        telemetry.addData(caption, message);
    }
    public void update() { telemetry.update(); }
    public double round(double val, int roundTo) {
        return Double.valueOf(String.format("%." + roundTo + "f", val));
    }
    public void blinkLED(int time) {
        if (gametime.milliseconds() > time && gametime.milliseconds() < time+1000) {
            led.setPattern(blinkPattern);
        } else {
            led.setPattern(pattern);
        }
    }

    // Init functions
    public void getPreferences() {
        preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamColor = String.valueOf(preferences.getString("auto_team_color", "blue"));
    }
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
        topLimit            = hardwareMap.get(DigitalChannel.class, "topLimit");
        bottomLimit         = hardwareMap.get(DigitalChannel.class, "bottomLimit");
        LogitechC310        = hardwareMap.get(WebcamName.class, "Logitech C310");
        led                 = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }
    private void initMotor() {
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gripMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up parameters for driving in a straight line.
        pidCorrection.setSetpoint(0);
        pidCorrection.setOutputRange(0, drivePower);
        pidCorrection.setInputRange(-90, 90);
        pidCorrection.enable();
    }
    private boolean checkMotor() {
        if (gripMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            return true;
        } else {
            return false;
        }
    }
    private void initServo() {
        leftServo.setPosition(0);
        rightServo.setPosition(1);
    }
    private boolean checkServo() {
        if (round(leftServo.getPosition(), 2) == 0 &&
                round(rightServo.getPosition(), 2) == 1) {
            return true;
        } else {
            return false;
        }
    }
    private void initIMU() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        while(!imu.isGyroCalibrated()){}
    }
    private boolean checkIMU() {
        if (imu.isGyroCalibrated()) {
            return true;
        } else {
            return false;
        }
    }
    public void initCheck() {
        while (!isStopRequested() && !initReady) {
            // Initialize motor
            if (!checkMotor()) {
                print("Motor","Initializing");
                update();
                initMotor();
            } else if (checkMotor()) {
                print("Motor","Initialized");
                update();
                // Initialize servo
                if (!checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initializing");
                    update();
                    initServo();
                } else if (checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initialized");
                    update();
                    // Initialize imu
                    if (!checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initializing...");
                        update();
                        initIMU();
                    } else if (checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initialized");
                        update();
                        initReady = true;
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
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    public void stopMotor() {
        run(0, 0, 0, 0);
    }
    public void stopAllMotors() {
        run(0, 0, 0, 0);
        armMotor.setPower(0);
        gripMotor.setPower(0);
    }
    private void pause(String mode) {
        int duration = 0;
        runtime.reset();
        runtime.startTime();
        switch (mode) {
            case "servo":
                duration = (int)(servoTimePer90Deg * 0.5);
                break;
            case "motor":
                duration = 250;
                break;
        }
        while (runtime.milliseconds() < duration){}
    }
    public boolean topPressed() {return !topLimit.getState();}
    public boolean bottomPressed() {return !bottomLimit.getState();}
}