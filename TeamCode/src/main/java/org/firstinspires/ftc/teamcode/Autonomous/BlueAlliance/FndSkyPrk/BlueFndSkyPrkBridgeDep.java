package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndSkyPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueFndSkyPrkBridgeDep", group = "FndSkyPrk")
public class BlueFndSkyPrkBridgeDep extends MainAutonomous {
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
            encoderDriveSmooth("front", 0.5);
            recognizeTarget("Stone Target");
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDriveSmoothDist("right", travelX);

            encoderDriveSmoothDist("front", travelY, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration*2);
            encoderDriveSmoothDist("back", travelY-0.5*inPerBlock);

            rotate(-90, turnPower);
            encoderDriveSmoothDist("front", 3*inPerBlock+travelX);
            rotate(90, turnPower);
            encoderDriveSmoothDist("left", firstPlacement);
            buildSkystone(1);

            // 2nd skystone
            if (numDelivered == 2) {
                rotate(90, turnPower);
                encoderDriveSmoothDist("front", 3 * inPerBlock + travelX);
                rotate(-90, turnPower);
                encoderDriveSmooth("right", 1);

                encoderDriveSmoothDist("front", travelY - 0.5 * inPerBlock, minPower);
                if (skystonePosition == "right") {
                    rotate(45, turnPower);
                }
                armDrop(armDuration);
                gripHold(gripDuration);
                armRaise(armDuration);
                if (skystonePosition == "right") {
                    rotate(-45, turnPower);
                }
                encoderDriveSmoothDist("back", travelY - 0.5 * inPerBlock);

                rotate(-90, turnPower);
                encoderDriveSmoothDist("front", 4 * inPerBlock + travelX);
                rotate(90, turnPower);

                encoderDriveSmoothDist("left", secondPlacement);
                buildSkystone(1);
            }
            // Foundation
            rotate(90, turnPower);
            rotate(90, turnPower);
            if (numDelivered == 1) {
                encoderDriveSmoothDist("right", (centerPlacement - firstPlacement));
            } else if (numDelivered == 2) {
                encoderDriveSmoothDist("right", (centerPlacement - secondPlacement));
            }
            encoderDriveSmooth("back",0.25+0.1);
            hookOn();
            encoderDriveSmooth("front", 0.5+0.1);
            curve(-90, turnPower);
            encoderDriveSmooth("back", 0.75);
            hookOff();
            // Parking
            encoderDriveSmoothDist("left", centerPlacement);
            armExtend();
            encoderDriveSmooth("front", 1.625);
            encoderDriveSmooth("left", 0.25, minPower);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
