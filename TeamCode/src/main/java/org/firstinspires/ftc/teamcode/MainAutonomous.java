package org.firstinspires.ftc.teamcode;

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

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

/*
References:
-Template: https://github.com/FestiveInvader/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/DeclarationsAutonomous.java
-Vuforia: https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVuforiaNavigationWebcam.java
-Gyro Correction: https://stemrobotics.cs.pdx.edu/node/7265
 */

@Autonomous
public class MainAutonomous extends LinearOpMode {
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
    public ElapsedTime runtime;

    // Declare movement variables
    private static final int ticksPerRev = 480;
    Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double correction;
    private double maxPower = 100/127;

    // Declare vuforia variables
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackable stoneTarget, blueRearBridge, redRearBridge, redFrontBridge,
            blueFrontBridge, red1, red2, front1, front2, blue1, blue2, rear1, rear2;
    private OpenGLMatrix lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
    private OpenGLMatrix latestLocation;
    private OpenGLMatrix webcamLocation;
    private static final String VUFORIA_KEY = "AZ2DQXn/////AAABmV2NdKltaEv7nZA9fnEAYpONbuK/sGbJG" +
            "7tGyKNwNcaEPXyRq7V3WKOcmTwGwpTyl5Sm/2tJR6t5VFwarUda2dnW20yakyCThxpQcM4xXu5xnY3/HVPc" +
            "TCEloelyqgf0jSbw94/N7b2n7jdkdA/CYYvJOQo7/cQ3cnoa/3aZ1LpJgeYy8SHLDeLe2nwpARjaHokhhG8" +
            "35GzpFlTXa1IhHjo0Lsvm2qTM8WqgLIKYYep1urYPAPYYUsT+WXUSLCbw0TkQcIVLP6FdvQL6FtCeRoA29f" +
            "pTdq5L4RFsdqac2fELdXY8rjZpJDx4g/8KN6aw1iG4ZocJBzgzhELtCgQbqJppGGk7z/CRTvcXL1dhIunZ";
    private int cameraMonitorViewId;
    private boolean targetVisible               = false;
    private boolean vuforiaReady                = false;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6.00f * mmPerInch;
    // Location of target with relation to the center of the field
    private static final float stoneZ           = 2.00f * mmPerInch;
    private static final float bridgeZ          = 6.42f * mmPerInch;
    private static final float bridgeY          = 23 * mmPerInch;
    private static final float bridgeX          = 5.18f * mmPerInch;
    private static final float bridgeRotY       = 59; // degrees
    private static final float bridgeRotZ       = 180; // degrees
    private static final float halfField        = 72 * mmPerInch;
    private static final float quadField        = 36 * mmPerInch;
    // Location of the robot with relation to the center of the field
    private float robotX                        = 0;
    private float robotY                        = 0;
    private float robotAngle                    = 0;
    // Location of the webcam with relation to the center of the robot
    private static final float webcamX          = 0;
    private static final float webcamY          = 6.5f * mmPerInch;
    private static final float webcamZ          = -3.0f * mmPerInch;
    private static final float webcamRotX       = 90; // degrees
    private static final float webcamRotY       = 0; // degrees
    private static final float webcamRotZ       = 180; // degrees

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize hardware
        getHardwareMap();
        initCheck();

        /*
        visionTargets.activate();

        while(!isStopRequested()) {
            targetVisible = false;
            // Check if any trackable target is visible
            for(VuforiaTrackable trackable : allTrackables) {
                if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    print("Visible Target", trackable.getName());
                    targetVisible = true;

                    latestLocation = ((VuforiaTrackableDefaultListener)trackable.getListener())
                            .getUpdatedRobotLocation();
                    if(latestLocation != null) lastKnownLocation = latestLocation;
                    break;
                }
            }

            // Update the robot's location with relation to the field
            if(targetVisible) {
                float [] coordinates = lastKnownLocation.getTranslation().getData();
                robotX      = coordinates[0];
                robotY      = coordinates[1];
                robotAngle  = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC,
                        AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                print("Robot Location", "(" + robotX + ", " + robotY + ") [" + robotAngle + "]");
            }
            else { print("Visible Target", "none"); }

            telemetry.update();
        }

        visionTargets.deactivate();
        */

        runtime.reset();
    }
    // Telemtry functions
    public void print(String caption, Object message) {
        telemetry.addData(caption, message);
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
        leftServo.setPosition(0.1);
        rightServo.setPosition(0.9);
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
        else {
            return false;
        }
    }
    public double round(double val, int roundTo) {
        return Double.valueOf(String.format("%." + roundTo + "f", val));
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
    public void initVuforia() {
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
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

        // Set up the field coordinate system

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

        vuforiaReady = true;
    }
    public boolean checkVuforia() {
        if (vuforiaReady) return true;
        else return false;
    }
    public void initCheck() {
        while (!isStopRequested() && (!(checkMotor() && checkServo() && checkIMU() && checkVuforia()))) {
            print("leftservo", round(leftServo.getPosition(), 2));
            print("rightservo", round(rightServo.getPosition(), 2));
             // Initialize motor
            if(!checkMotor()) {
                print("Motor","Initializing");
                initMotor();
            }
            else if(checkMotor()) {
                print("Motor","Initialized");
                // Initialize servo
                if(!checkServo()) {
                    print("Servo","Initializing");
                    initServo();
                }
                else if(checkServo()) {
                    print("Servo","Initialized");
                    // Initialize imu
                    if(!checkIMU()) {
                        print("IMU","Initializing...");
                        initIMU();
                    }
                    else if(checkIMU()) {
                        print("IMU","Initialized");
                        // Initialize vuforia
                        if(!checkVuforia()) {
                            print("Vuforia","Initializing...");
                            initVuforia();
                        }
                        else if(checkVuforia()) {
                            print("Vuforia","Initialized");
                        }
                    }
                }
            }
            telemetry.update();
            idle();
            sleep(50);
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

    // Movement functions
    public void run(double leftBackPower, double rightBackPower, double leftFrontPower, double rightFrontPower) {
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
    }
    public void gyroRun() {
        while (opModeIsActive()) {
            correction = checkDirection();
            if (Math.abs(correction) > 0 ) {

            }
        }
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    private double checkDirection() {
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
}
