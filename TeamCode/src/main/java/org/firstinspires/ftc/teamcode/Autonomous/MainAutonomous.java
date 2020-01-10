package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.ftccommon.SoundPlayer;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import java.util.ArrayList;
import java.util.List;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

/*
References:
-Template: https://github.com/FestiveInvader/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/DeclarationsAutonomous.java
-Vuforia: https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVuforiaNavigationWebcam.java
-Gyro Correction: https://stemrobotics.cs.pdx.edu/node/7265
 */

@Autonomous
public class MainAutonomous extends LinearOpMode {
    /*
    ======================================== DECLARE VARIABLES =========================================
     */
    // Declare hardware variables
    public BNO055IMU imu;
    public DcMotor leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor, armMotor, gripMotor;
    public Servo leftServo, rightServo, leftSkystoneServo, rightSkystoneServo;
    public ColorSensor leftColorSensor, rightColorSensor;
    public DigitalChannel topLimit, bottomLimit;
    public WebcamName LogitechC310;

    // Declare general variables
    public ElapsedTime runtime = new ElapsedTime();
    public boolean initReady = false;
    public static float inPerBlock = 23.625f;
    public static float fullSkystoneDist = 8.0f;
    public static float halfSkystoneDist = 4.0f;
    public double leftColorThreshold, rightColorThreshold;

    // Declare movement variables
    public static final int ticksPerRev = 480;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    public double correction;
    public double minPower = 0.25;
    public double maxPower = round((100.0/127.0), 2);
    public double minTurnPower = 0.35;
    public double maxTurnPower = round((90.0/127.0), 2);
    public int timePerBlock = 1350;
    public int timePer90Deg = 1000;

    // Declare vuforia variables
    public VuforiaLocalizer vuforia;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables visionTargets;
    public List<VuforiaTrackable> allTrackables;
    public VuforiaTrackable stoneTarget, blueRearBridge, redRearBridge, redFrontBridge,
            blueFrontBridge, red1, red2, front1, front2, blue1, blue2, rear1, rear2;
    public OpenGLMatrix lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    public OpenGLMatrix latestLocation;
    public OpenGLMatrix webcamLocation;
    public String skystonePosition;
    public static final String VUFORIA_KEY = "AZ2DQXn/////AAABmV2NdKltaEv7nZA9fnEAYpONbuK/sGbJG" +
            "7tGyKNwNcaEPXyRq7V3WKOcmTwGwpTyl5Sm/2tJR6t5VFwarUda2dnW20yakyCThxpQcM4xXu5xnY3/HVPc" +
            "TCEloelyqgf0jSbw94/N7b2n7jdkdA/CYYvJOQo7/cQ3cnoa/3aZ1LpJgeYy8SHLDeLe2nwpARjaHokhhG8" +
            "35GzpFlTXa1IhHjo0Lsvm2qTM8WqgLIKYYep1urYPAPYYUsT+WXUSLCbw0TkQcIVLP6FdvQL6FtCeRoA29f" +
            "pTdq5L4RFsdqac2fELdXY8rjZpJDx4g/8KN6aw1iG4ZocJBzgzhELtCgQbqJppGGk7z/CRTvcXL1dhIunZ";
    public int cameraMonitorViewId;
    public boolean streamView                   = false;
    public boolean targetVisible                = false;
    public boolean vuforiaReady                 = false;
    public boolean skystoneFound                = false;
    public static final float mmPerInch         = 25.4f;
    public static final float mmTargetHeight    = 6.00f * mmPerInch;
    // Location of center of target with relation to center of field
    public static final float stoneZ            = 2.00f * mmPerInch;
    public static final float bridgeZ           = 6.42f * mmPerInch;
    public static final float bridgeY           = 23 * mmPerInch;
    public static final float bridgeX           = 5.18f * mmPerInch;
    public static final float bridgeRotY        = 59; // degrees
    public static final float bridgeRotZ        = 180; // degrees
    public static final float halfField         = 72 * mmPerInch;
    public static final float quadField         = 36 * mmPerInch;
    // Location of center of robot with relation to center of target
    public float robotX                         = 0;
    public float robotY                         = 0;
    public float robotAngle                     = 0;
    // Location of center of webcam with relation to center of robot
    public static final float webcamX           = 0;
    public static final float webcamY           = 6.50f * mmPerInch;
    public static final float webcamZ           = -3.00f * mmPerInch;
    public static final float webcamRotX        = 90; // degrees
    public static final float webcamRotY        = 0; // degrees
    public static final float webcamRotZ        = 180; // degrees
    // Dimension of robot
    public float robotWidth                     = 18.00f;
    public float robotLength                    = 18.00f;
    public float robotHeight                    = 14.00f;
    // Distance to travel from front of robot to center of target
    public float travelX                        = fullSkystoneDist + halfSkystoneDist - (robotWidth / 2);
    public float travelY                        = inPerBlock;
    // Location of center of skystone placement with relation to wall
    public float firstSkystone                  = 13.25f;
    public float secondSkystone                 = 29.25f;
    public float centerSkystone                 = 21.25f;
    // Distance to travel from building site starting position to center of skystone placement (towards wall)
    public float firstPlacement                 = inPerBlock - firstSkystone + (robotWidth / 2);
    public float secondPlacement                = inPerBlock - secondSkystone + (robotWidth / 2);
    public float centerPlacement                = inPerBlock - centerSkystone + (robotWidth / 2);

    // Declare shared preference variables
    public SharedPreferences preferences;
    public String teamColor, parking, starting;
    public boolean doFoundation, doSkystone;
    public int delayTime;
    public StringBuilder autoName = new StringBuilder("");

    /*
    ======================================= AUTONOMOUS PROGRAM =========================================
     */
    @Override
    public void runOpMode() {
        encoderDrive("front", minPower, 1);
        encoderDrive("back", minPower, 1);
        gyroTurn(90, minTurnPower);
        gyroTurn(-90, minTurnPower);
    }
    /*
    ======================================== DECLARE FUNCTIONS =========================================
     */
    // General functions
    public void print(String caption, Object message) {
        telemetry.addData(caption, message);
    }
    public double round(double val, int roundTo) {
        return Double.valueOf(String.format("%." + roundTo + "f", val));
    }
    public void pause() {sleep(150);}
    public void playSound(String soundName) {
        // List of available sound resources
        String [] sounds = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
                "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
                "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};

        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        int sound = hardwareMap.appContext.getResources().getIdentifier(soundName, "raw", hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, sound, params, null, null);
    }

    // Init functions
    public void getPreferences() {
        preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamColor = String.valueOf(preferences.getString("auto_team_color", "blue"));
        doFoundation = Boolean.valueOf(preferences.getBoolean("auto_do_foundation", true));
        doSkystone = Boolean.valueOf(preferences.getBoolean("auto_do_skystone", false));
        parking = String.valueOf(preferences.getString("auto_parking", "bridge"));
        starting = String.valueOf(preferences.getString("auto_starting", "depot"));
        delayTime = Integer.valueOf(preferences.getString("auto_delay_time", "0"));
    }
    public void checkPreferences(String className) {
        if (teamColor == "blue") autoName.append("Blue");
        else if (teamColor == "red") autoName.append("Red");
        if (doFoundation) autoName.append("Fnd");
        if (doSkystone) autoName.append("Sky");
        if (parking == "bridge") autoName.append("Bridge");
        else if (parking == "wall") autoName.append("Wall");
        if (starting == "depot") autoName.append("Dep");
        else if (starting == "buildingSite") autoName.append("Build");

        if (className != autoName.toString()) {
            AppUtil.getInstance().showToast(UILocation.BOTH, "Selected Autonomous mode - \'" + autoName + "\'", 5000);
            requestOpModeStop();
        }
        else AppUtil.getInstance().showToast(UILocation.BOTH, "Selected Autonomous mode - \'" + autoName + "\'", 5000);
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
        leftSkystoneServo   = hardwareMap.get(Servo.class, "leftSkystoneServo");
        rightSkystoneServo  = hardwareMap.get(Servo.class, "rightSkystoneServo");
        leftColorSensor     = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor    = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        topLimit            = hardwareMap.get(DigitalChannel.class, "topLimit");
        bottomLimit         = hardwareMap.get(DigitalChannel.class, "bottomLimit");
        LogitechC310        = hardwareMap.get(WebcamName.class, "Logitech C310");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }
    public void initMotor() {
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
        while(!imu.isGyroCalibrated()){}
    }
    public boolean checkIMU() {
        if (imu.isGyroCalibrated()) return true;
        else return false;
    }
    public void initVuforia() {
        if (streamView) parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        else parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = LogitechC310;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for trackable objects
        visionTargets = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = visionTargets.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = visionTargets.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = visionTargets.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = visionTargets.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = visionTargets.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = visionTargets.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = visionTargets.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = visionTargets.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = visionTargets.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = visionTargets.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = visionTargets.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = visionTargets.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = visionTargets.get(12);
        rear2.setName("Rear Perimeter 2");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(visionTargets);

        // Set up field coordinate system

        // Field Coordinate System
        // If you are standing in the Red Alliance Station looking towards the center of the field:
        // -The X axis runs from your left to the right. (Positive is from the center to the right)
        // -The Y axis runs from the Red Alliance Station towards the Blue Alliance Station
        //  (Positive is from the center to the Blue Alliance Station)
        // -The Z axis runs from the floor to the ceiling. (Positive is above the floor)

        // Location of the Stone Target
        stoneTarget.setLocation(createMatrix(0, 0, stoneZ, 90, 0, -90));
        // Location of the bridge support targets with relation to the center of field
        blueFrontBridge.setLocation(createMatrix(-bridgeX, bridgeY, bridgeZ, 0, bridgeRotY, bridgeRotZ));
        blueRearBridge.setLocation(createMatrix(-bridgeX, bridgeY, bridgeZ, 0, -bridgeRotY, bridgeRotZ));
        redFrontBridge.setLocation(createMatrix(-bridgeX, -bridgeY, bridgeZ, 0, -bridgeRotY, 0));
        redRearBridge.setLocation(createMatrix(bridgeX, -bridgeY, bridgeZ, 0, bridgeRotY, 0));
        // Location of the perimeter targets with relation to the center of field
        red1.setLocation(createMatrix(quadField, -halfField, mmTargetHeight, 90, 0, 180));
        red2.setLocation(createMatrix(-quadField, -halfField, mmTargetHeight, 90, 0, 180));
        front1.setLocation(createMatrix(-halfField, -quadField, mmTargetHeight, 90, 0 , 90));
        front2.setLocation(createMatrix(-halfField, quadField, mmTargetHeight, 90, 0, 90));
        blue1.setLocation(createMatrix(-quadField, halfField, mmTargetHeight, 90, 0, 0));
        blue2.setLocation(createMatrix(quadField, halfField, mmTargetHeight, 90, 0, 0));
        rear1.setLocation(createMatrix(halfField, quadField, mmTargetHeight, 90, 0 , -90));
        rear2.setLocation(createMatrix(halfField, -quadField, mmTargetHeight, 90, 0, -90));
        // Location of the webcam with relation to the center of the robot
        webcamLocation = createMatrix(webcamX, webcamY, webcamZ, webcamRotX, webcamRotY, webcamRotZ);

        // Set up the listener
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener())
                    .setCameraLocationOnRobot(parameters.cameraName, webcamLocation);
        }

        visionTargets.activate();
        vuforiaReady = true;
    }
    public boolean checkVuforia() {
        if (vuforiaReady) return true;
        else return false;
    }
    public void initCheck() {
        while (!isStopRequested() && !(checkMotor() && checkServo() && checkIMU() && checkVuforia())) {
            // Initialize motor
            if (!checkMotor()) {
                print("Motor","Initializing");
                telemetry.update();
                initMotor();
            }
            else if (checkMotor()) {
                print("Motor","Initialized");
                telemetry.update();
                // Initialize servo
                if (!checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initializing");
                    telemetry.update();
                    initServo();
                }
                else if (checkServo()) {
                    print("Motor","Initialized");
                    print("Servo","Initialized");
                    telemetry.update();
                    // Initialize imu
                    if (!checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initializing...");
                        telemetry.update();
                        initIMU();
                    }
                    else if (checkIMU()) {
                        print("Motor","Initialized");
                        print("Servo","Initialized");
                        print("IMU","Initialized");
                        telemetry.update();
                        // Initialize vuforia
                        if (!checkVuforia()) {
                            print("Motor","Initialized");
                            print("Servo","Initialized");
                            print("IMU","Initialized");
                            print("Vuforia","Initializing...");
                            telemetry.update();
                            initVuforia();
                        }
                        else if (checkVuforia()) {
                            print("Motor","Initialized");
                            print("Servo","Initialized");
                            print("IMU","Initialized");
                            print("Vuforia","Initialized");
                            telemetry.update();
                            initReady = true;
                        }
                    }
                }
            }
        }
    }

    // Vuforia functions
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix
                .translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    public String formatMatrix(OpenGLMatrix matrix) {return matrix.formatAsTransform();}
    public void recognizeTarget(String target) {
        boolean targetFound = false;
        while(opModeIsActive() && checkVuforia()) {
            String targetName = "";
            targetVisible = false;
            skystoneFound = false;
            // Check if any trackable target is visible
            for(VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    targetName = trackable.getName();
                    print("Visible Target", targetName);
                    if (targetName == "Stone Target") skystoneFound = true;

                    latestLocation = ((VuforiaTrackableDefaultListener)trackable.getListener())
                            .getUpdatedRobotLocation();
                    if (latestLocation != null) lastKnownLocation = latestLocation;
                    break;
                }
            }

            if (targetVisible) {
                float [] coordinates = lastKnownLocation.getTranslation().getData();
                robotX      = coordinates[1] / mmPerInch;
                robotY      = coordinates[0] / mmPerInch;
                robotAngle  = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC,
                        AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                // Update robot's location with relation to center of target
                print("Robot Coordinates", "(" + round(robotX, 0) + "in , " + round(robotY, 0) + "in)");
                print("Robot Heading", round(robotAngle, 2));

                // Update travelling distance
                if (robotX != 0) travelX = robotX;
                if (robotY != 0) travelY = robotY - (robotLength / 2);

                if (skystoneFound) {
                    if (robotX < -0.4) skystonePosition = "left";
                    else skystonePosition = "center";
                }
            }
            else {
                print("Visible Target", "none");
                skystonePosition = "right";
            }

            // Update the skystone's position in terms of "left", "center", and "right"
            print("Skystone Position", skystonePosition);

            telemetry.update();

            if (targetName == target) break;
        }
    }

    // Movement functions
    public void run(double leftBackPower, double rightBackPower, double leftFrontPower, double rightFrontPower) {
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
    }
    public void set(int leftBackDistance, int rightBackDistance, int leftFrontDistance, int rightFrontDistance) {
        leftBackMotor.setTargetPosition(leftBackDistance);
        rightBackMotor.setTargetPosition(rightBackDistance);
        leftFrontMotor.setTargetPosition(leftFrontDistance);
        rightFrontMotor.setTargetPosition(rightFrontDistance);
    }
    public void timeDrive(String direction, double power, int duration) {
        // The factor value determines the scaling factor to normalize each movement
        double axialFactor    = 1.0;
        double lateralFactor      = 2.0;
        double diagonalFactor    = 3.25;
        double axialPower     = round((power * axialFactor), 2);
        double lateralPower       = round((power * lateralFactor), 2);
        double diagonalPower     = round((power * diagonalFactor), 2);
        runtime.reset();
        runtime.startTime();
        switch (direction) {
            case "front":
                run(axialPower, axialPower, axialPower, axialPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "back":
                run(-axialPower, -axialPower, -axialPower, -axialPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "left":
                run(lateralPower, -lateralPower, -lateralPower, lateralPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "right":
                run(-lateralPower, lateralPower, lateralPower, -lateralPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "front left":
                run(diagonalPower, 0, 0, diagonalPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "front right":
                run(0, diagonalPower, diagonalPower, 0);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "back left":
                run(0, -diagonalPower, -diagonalPower, 0);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "back right":
                run(-diagonalPower, 0, 0, -diagonalPower);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
        }
        stopMotor();
    }
    public void timeTurn(String direction, double power, int duration) {
        runtime.reset();
        runtime.startTime();
        switch (direction) {
            case "clockwise":
                run(power, -power, power, -power);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
            case "counter-clockwise":
                run(-power, power, -power, power);
                while(opModeIsActive() && runtime.milliseconds() < duration) {}
        }
        resetAngle();
        stopMotor();
    }
    public void encoderDrive(String direction, double power, double block) {
        double distance = block * inPerBlock;
        encoderDriveDist(direction, power, distance);
    }
    public void encoderDriveDist(String direction, double power, double dist) {
        if (power < minPower) power = minPower;
        else if (power > maxPower) power = maxPower;

        double circumference    = Math.PI * 3.75;
        double inPerRev         = circumference / ticksPerRev;
        int axialDistance       = (int)(dist / inPerRev);
        int lateralDistance     = (int)(dist * Math.sqrt(2) / inPerRev);
        int diagonalDistance    = (int)(dist * Math.sqrt(2) / inPerRev);
        int maxRuntime          = (int)(timePerBlock * dist / 24);

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (direction == "front") { set(axialDistance, axialDistance, axialDistance, axialDistance); }
        if (direction == "back") { set(-axialDistance, -axialDistance, -axialDistance, -axialDistance); }
        if (direction == "left") { set(lateralDistance, -lateralDistance, -lateralDistance, lateralDistance); }
        if (direction == "right") { set(-lateralDistance, lateralDistance, lateralDistance, -lateralDistance); }
        if (direction == "front left") { set(diagonalDistance, 0, 0, diagonalDistance); }
        if (direction == "front right") { set(0, diagonalDistance, diagonalDistance, 0); }
        if (direction == "back left") { set(0, -diagonalDistance, -diagonalDistance, 0); }
        if (direction == "back right") { set(-diagonalDistance, 0, 0, -diagonalDistance); }

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        runtime.startTime();
        run(power, power, power, power);
        while(opModeIsActive() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && runtime.milliseconds() < maxRuntime) {
            telemetry.addData("axialdist",axialDistance);
            telemetry.addData("lateraldist",lateralDistance);
            telemetry.addData("diagonalDist",diagonalDistance);
            telemetry.addData("backleft", leftBackMotor.getCurrentPosition());
            telemetry.addData("backright", rightBackMotor.getCurrentPosition());
            telemetry.addData("frontleft", leftFrontMotor.getCurrentPosition());
            telemetry.addData("frontright", rightFrontMotor.getCurrentPosition());
            telemetry.update();
        }
        stopMotor();
    }
    public void testEncoderDrive(String direction, double power, double block) {
        double circumference    = Math.PI * 3.75;
        double inPerRev         = circumference / ticksPerRev;
        int distance            = (int)(block * inPerBlock / inPerRev);

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (direction == "front") { set(distance, distance, distance, distance); }
        if (direction == "back") { set(-distance, -distance, -distance, -distance); }
        if (direction == "left") { set(distance, -distance, -distance, distance); }
        if (direction == "right") { set(-distance, distance, distance, -distance); }
        if (direction == "front left") { set(distance, 0, 0, distance); }
        if (direction == "front right") { set(0, distance, distance, 0); }
        if (direction == "back left") { set(0, -distance, -distance, 0); }
        if (direction == "back right") { set(-distance, 0, 0, -distance); }
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        run(power, power, power, power);
        while(opModeIsActive() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            telemetry.addData("backleft", leftBackMotor.getCurrentPosition());
            telemetry.addData("backright", rightBackMotor.getCurrentPosition());
            telemetry.addData("frontleft", leftFrontMotor.getCurrentPosition());
            telemetry.addData("frontright", rightFrontMotor.getCurrentPosition());
            telemetry.update();
        }
        stopMotor();
    }
    // TODO
    public void encoderDriveSmooth(int distance) {
        // The acceleration value determines the range to normalize each movement
        double accelerationRange    = 0.25;
        int accelerateDistance      = (int)Math.abs(distance * accelerationRange);
        int decelerateDistance      = (int)Math.abs(distance * (1-accelerationRange));

        int currentDistance         = (int)Math.abs(leftBackMotor.getCurrentPosition());
        double accelerationPower;
        double decelerationPower;

        run(minPower, minPower, minPower, minPower);
        while(opModeIsActive() && leftBackMotor.isBusy()
                && currentDistance < accelerateDistance) {
            accelerationPower = minPower + (currentDistance/accelerateDistance) * (maxPower - minPower);
            run (accelerationPower, accelerationPower, accelerationPower, accelerationPower);
        }
        while(opModeIsActive() && leftBackMotor.isBusy()
                && (currentDistance > accelerateDistance && currentDistance < decelerateDistance)) {
            run (maxPower, maxPower, maxPower, maxPower);
        }
        while(opModeIsActive() && leftBackMotor.isBusy()
                && currentDistance > decelerateDistance) {
            decelerationPower = maxPower - ((currentDistance-decelerateDistance)/(distance-decelerateDistance)) * (maxPower - minPower);
            run (decelerationPower, decelerationPower, decelerationPower, decelerationPower);
        }
        stopMotor();
        gyroCorrection();
    }
    public void gyroTurn(int angle, double power) {
        if (power < minTurnPower) power = minTurnPower;
        else if (power > maxTurnPower) power = maxTurnPower;

        double leftPower, rightPower;
        double fixScale = 0.13888;
        int maxRuntime = (int)(timePer90Deg * angle / 90);

        resetAngle();

        if (angle < 0) {
            leftPower = -power;
            rightPower = power;
        }
        else if (angle > 0) {
            leftPower = power;
            rightPower = -power;
        }
        else return;

        runtime.reset();
        runtime.startTime();
        run(leftPower, rightPower, leftPower, rightPower);

        if (angle < 0) {
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > angle + angle * fixScale && runtime.milliseconds() < maxRuntime) {}
        }
        else {
            while (opModeIsActive() && getAngle() < angle - angle * fixScale && runtime.milliseconds() < maxRuntime) {}
        }
        resetAngle();
        stopMotor();
    }
    // TODO
    public void gyroCurve(int angle, double power) {
        if (power < minTurnPower) power = minTurnPower;
        else if (power > maxTurnPower) power = maxTurnPower;

        double leftPower, rightPower;
        double gainPower = power * (Math.sqrt(2) - 1);
        double fixScale = 0.13888;

        resetAngle();

        if (angle < 0) {
            leftPower = -power;
            rightPower = power + 2 * gainPower;
        }
        else if (angle > 0) {
            leftPower = -power;
            rightPower = -power + 2 * gainPower;
        }
        else return;

        run(leftPower, rightPower, leftPower, rightPower);

        if (angle < 0) {
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > angle + angle * fixScale) {}
        }
        else {
            while (opModeIsActive() && getAngle() < angle - angle * fixScale) {}
        }
        resetAngle();
        stopMotor();
    }
    public void gyroCorrection() {
        runtime.reset();
        runtime.startTime();
        while (opModeIsActive() && checkGyro()) {
            if (getAngle() < 0) run(-minPower, minPower, -minPower, minPower);
            else if (getAngle() > 0) run(minPower, -minPower, minPower, -minPower);
        }
        stopMotor();
    }
    public boolean checkGyro() {
        if (Math.abs(getAngle()) < 1) return false;
        else if (Math.abs(getAngle()) < 3 && runtime.milliseconds() > 1000) return false;
        else return true;
    }
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return -globalAngle;
    }
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        double correction;
        double angle = getAngle();
        double gain = 0.025;

        if (angle == 0) correction = 0;
        else correction = -angle;
        correction = correction * gain;

        return correction;
    }
    public void stopMotor() {
        run(0, 0, 0, 0);
    }
    public void stopAllMotors() {
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        armMotor.setPower(0);
        gripMotor.setPower(0);
    }
    public boolean topPressed() {
        return !topLimit.getState();
    }
    public boolean bottomPressed() {
        return !bottomLimit.getState();
    }
    public void armExtend() {
        armMotor.setPower(-0.5);
        while (opModeIsActive() && !bottomPressed()) {}
        armMotor.setPower(0);
    }
    public void armCollapse() {
        armMotor.setPower(0.5);
        while (opModeIsActive() && !topPressed()) {}
        armMotor.setPower(0);
    }
    public void armRaise(int duration) {
        runtime.reset();
        runtime.startTime();
        armMotor.setPower(0.5);
        while(opModeIsActive() && !topPressed() && runtime.milliseconds() < duration) {}
        armMotor.setPower(0);
    }
    public void armDrop(int duration) {
        runtime.reset();
        runtime.startTime();
        armMotor.setPower(-0.5);
        while(opModeIsActive() && !bottomPressed() && runtime.milliseconds() < duration) {}
        armMotor.setPower(0);
    }
    public void gripHold(int duration) {
        runtime.reset();
        runtime.startTime();
        if (!topPressed()) gripMotor.setPower(0.25);
        while(opModeIsActive() && !topPressed() && runtime.milliseconds() < duration) {}
    }
    public void gripRelease(int duration) {
        runtime.reset();
        runtime.startTime();
        if (!topPressed()) gripMotor.setPower(-0.25);
        while(opModeIsActive() && !topPressed() && runtime.milliseconds() < duration) {}
        gripMotor.setPower(0);
    }
    public void hookOn() {
        leftServo.setPosition(1);
        rightServo.setPosition(0);
        while(opModeIsActive() && round(leftServo.getPosition(), 2) != 1 && round(rightServo.getPosition(), 2) != 0) {}
    }
    public void hookOff() {
        leftServo.setPosition(0.10);
        rightServo.setPosition(0.90);
        while(opModeIsActive() && round(leftServo.getPosition(), 2) != 0.10 && round(rightServo.getPosition(), 2) != 0.90) {}
    }
    public void depotToBuildingSite(String alliance, double power, double turnPower, double block) {
        switch (alliance) {
            case "blue":
                gyroTurn(-90, turnPower);
                encoderDrive("front", power, block);
                gyroTurn(90, turnPower);
            case "red":
                gyroTurn(90, turnPower);
                encoderDrive("front", power, block);
                gyroTurn(-90, turnPower);
        }
    }
    public void buildingSiteToDepot(String alliance, double power, double turnPower, double block) {
        switch (alliance) {
            case "blue":
                gyroTurn(90, turnPower);
                encoderDrive("front", power, block);
                gyroTurn(-90, turnPower);
            case "red":
                gyroTurn(-90, turnPower);
                encoderDrive("front", power, block);
                gyroTurn(90, turnPower);
        }
    }
    public void grabFoundation(String alliance) {
        gyroTurn(180, minTurnPower);
        encoderDrive("back", minPower, 0.25);
        hookOn();
        encoderDrive("back", minPower, 0.5);
        if (alliance == "blue") gyroCurve(-90, minTurnPower);
        else gyroCurve(90, minTurnPower);
        encoderDrive("back", minPower, 0.75);
        hookOff();
    }
    public void recognizeSkystone(String alliance) {
        switch (alliance) {
            case "blue":
                leftColorThreshold = leftColorSensor.red() * leftColorSensor.green() / Math.pow(leftColorSensor.blue(), 2);
                if (leftColorThreshold <= 3) captureLeftSkystone();
            case "red":
                rightColorThreshold = rightColorSensor.red() * rightColorSensor.green() / Math.pow(rightColorSensor.blue(), 2);
                if (rightColorThreshold <= 3) captureRightSkystone();
        }
    }
    public void sideGrabSkystone(String alliance) {
        if (alliance == "blue") {
            switch (skystonePosition) {
                case "left":
                    travelX = 2 * fullSkystoneDist;
                case "center":
                    travelX = fullSkystoneDist;
            }
            encoderDriveDist("back", minPower, travelX);
            encoderDrive("left", minPower, 0.25);
            captureLeftSkystone();
            encoderDrive("right", minPower, 0.25);
        }
        else if (alliance == "red") {
            switch (skystonePosition) {
                case "right":
                    travelX = 2 * fullSkystoneDist;
                case "center":
                    travelX = fullSkystoneDist;
            }
            encoderDriveDist("back", minPower, travelX);
            encoderDrive("right", minPower, 0.25);
            encoderDrive("left", minPower, 0.25);
            captureRightSkystone();
        }
    }
    public void grabSkystone(double power) {
        armExtend();
        gripRelease(150);
        encoderDriveDist("front", power, travelY);
        gripHold(300);
        encoderDrive("back", power, 0.25);
    }
    public void buildSkystone(String alliance, double power, int height) {
        int duration = 300 * height;
        armRaise(duration);
        encoderDrive("front", minPower, 0.25);
        gripRelease(150);
        encoderDrive("back", minPower, 0.25);
        armDrop(duration);
    }
    public void captureLeftSkystone() {
        leftSkystoneServo.setPosition(0.98);
        while(opModeIsActive() && round(leftSkystoneServo.getPosition(), 2) != 0.98) {}
    }
    public void captureRightSkystone() {
        rightSkystoneServo.setPosition(0.52);
        while(opModeIsActive() && round(rightSkystoneServo.getPosition(), 2) != 0.52) {}
    }
}