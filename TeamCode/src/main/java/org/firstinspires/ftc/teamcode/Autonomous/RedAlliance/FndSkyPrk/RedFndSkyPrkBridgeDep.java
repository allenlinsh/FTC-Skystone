package org.firstinspires.ftc.teamcode.Autonomous.RedAlliance.FndSkyPrk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "RedFndSkyPrkBridgeDep", group = "FndSkyPrk")
public class RedFndSkyPrkBridgeDep extends MainAutonomous {
    public String className = getClass().getSimpleName();
    public int numDelivered = 1;
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
        update();

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

        waitForStart();
        print("Status", "Running");
        update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                Yes
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
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
            armRaise(armDuration*4);
            encoderDriveSmoothDist("back", travelY - (1-detectionTransition)*inPerBlock + skystoneMovementError*inPerBlock);

            // depot to building site
            rotate(90, turnPower);
            encoderDriveSmoothDist("front", 3*inPerBlock + travelX);
            rotate(-90, turnPower);
            encoderDriveSmoothDist("right", firstPlacement);
            buildSkystone(1);

            // 2nd skystone
            if (numDelivered == 2) {
                // building site to depot
                rotate(-90, turnPower);
                encoderDriveSmoothDist("front", 3 * inPerBlock + travelX);
                rotate(90, turnPower);
                encoderDriveSmooth("left", 1);

                // grab skystone
                encoderDriveSmoothDist("front", travelY - 0.5 * inPerBlock + skystoneMovementError*inPerBlock, minPower);
                if (skystonePosition == "left") {
                    rotate(-15, turnPower);
                }
                armDrop(armDuration);
                gripHold(gripDuration);
                armRaise(armDuration);
                if (skystonePosition == "left") {
                    rotate(15, turnPower);
                }
                encoderDriveSmoothDist("back", travelY - 0.5 * inPerBlock + skystoneMovementError*inPerBlock);

                // depot to building site
                if (skystonePosition == "left") {
                    rotate(15, turnPower);
                    rotate(15, turnPower);
                    rotate(15, turnPower);
                    rotate(15, turnPower);
                    rotate(15, turnPower);
                    rotate(10, turnPower);
                } else {
                    rotate(90, turnPower);
                }
                encoderDriveSmoothDist("front", 4 * inPerBlock + travelX);
                rotate(-90, turnPower);
                encoderDriveSmoothDist("right", secondPlacement);
                buildSkystone(1);
            }
            // Foundation
            rotate(90, turnPower);
            rotate(90, turnPower);
            if (numDelivered == 1) {
                encoderDriveSmoothDist("left", (centerPlacement - firstPlacement));
            } else if (numDelivered == 2) {
                encoderDriveSmoothDist("left", (centerPlacement - secondPlacement));
            }
            encoderDriveSmooth("back",(1-detectionTransition) + foundationMovementError);
            hookOn();
            encoderDriveSmooth("front", 1.25 + foundationMovementError);
            curve(90, turnPower);
            encoderDriveSmooth("back", 0.5);
            hookOff();
            // Parking
            timeDrive("front", minPower, 100);
            encoderDriveSmoothDist("right", centerPlacement + 0.5*inPerBlock);
            armExtend();
            encoderDriveSmooth("front", 1.625);
            encoderDriveSmooth("right", 0.25, minPower);

            stopAllMotors();
            visionTargets.deactivate();
        }
    }
}