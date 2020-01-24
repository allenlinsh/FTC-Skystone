package org.firstinspires.ftc.teamcode.Autonomous.RedAlliance.SkyPrk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "RedSkyPrkBridgeDep", group = "SkyPrk")
public class RedSkyPrkBridgeDep extends MainAutonomous {
    private String className = getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences(className);

        // Initialize hardware
        getHardwareMap();
        initCheck();
        print("Status", "Initialized");
        print("Alliance", teamColor);
        print("Foundation", doFoundation);
        print("Skystone", doSkystone);
        print("Parking Position", parking);
        print("Starting Position", starting);
        telemetry.update();

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                Yes
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Recognize skystone
            gripRelease(gripDuration/2);
            playSound("ss_power_up");
            encoderDriveSmooth("front", detectionTransition);
            recognizeTarget("Stone Target");
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDriveSmoothDist("left", travelX);

            // grab skystone
            armExtend();
            armRaise(armDuration);
            gripRelease(gripDuration);
            encoderDriveSmoothDist("front", travelY, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmoothDist("back", travelY - (1-detectionTransition)*inPerBlock + skystoneMovementError*inPerBlock, minPower);

            // depot to building site
            rotate(90, turnPower);
            encoderDriveSmoothDist("front", 2*inPerBlock + travelX);
            dropSkystone();

            // 2nd skystone
            // building site to depot
            encoderDriveSmoothDist("back", 2*inPerBlock + travelX + leftTranslation);
            rotate(-90, turnPower);
            encoderDriveSmooth("left", 1);

            // grab skystone
            encoderDriveSmoothDist("front", travelY - 0.5*inPerBlock + (1-detectionTransition)*inPerBlock + skystoneMovementError*inPerBlock, minPower);
            if (skystonePosition == "left") {
                rotate(-15, turnPower);
            }
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            if (skystonePosition == "left") {
                rotate(15, turnPower);
            }
            encoderDriveSmoothDist("back", travelY - (1-detectionTransition)*inPerBlock + skystoneMovementError*inPerBlock, minPower);

            // depot to building site
            if (skystonePosition == "right") {
                rotate(15, turnPower);
                rotate(15, turnPower);
                rotate(15, turnPower);
                rotate(15, turnPower);
                rotate(15, turnPower);
                rotate(15, turnPower);
            } else {
                rotate(90, turnPower);
            }
            encoderDriveSmoothDist("front", 3*inPerBlock + travelX);
            dropSkystone();
            encoderDriveSmooth("back", 0.625);
            encoderDriveSmooth("left", 0.25);

            stopAllMotors();
            visionTargets.deactivate();
        }
    }
}
