package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndSkyPrk;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BlueFndSkyPrkBridgeDep extends MainAutonomous {
    public String className = getClass().getSimpleName();
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

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                Yes
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Recognize skystone
            playSound("ss_power_up");
            encoderDrive("left", minPower, 0.25);
            encoderDrive("front", minPower, 0.25);
            recognizeTarget("Stone Target");
            // 1st skystone
            encoderDriveDist("right", minPower, travelX);
            grabSkystone(minPower);
            playSound("ss_roger_roger");
            depotToBuildingSite("blue", minPower, minTurnPower, 3);
            encoderDriveDist("left", minPower, (travelX + firstPlacement));
            buildSkystone("blue", minPower, 1);
            // 2nd skystone
            buildingSiteToDepot("blue", minPower, minTurnPower, 4);
            encoderDriveDist("right", minPower, (travelX + firstPlacement));
            grabSkystone(minPower);
            playSound("ss_roger_roger");
            depotToBuildingSite("blue", minPower, minTurnPower, 4);
            encoderDriveDist("left", minPower, (travelX + secondPlacement));
            buildSkystone("blue", minPower, 1);
            // Foundation
            encoderDriveDist("left", minPower, (centerPlacement - secondPlacement));
            grabFoundation("blue");
            // Parking
            encoderDrive("left", minPower, 0.75);
            encoderDrive("front", minPower, 1.625);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
