package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.SkystonePark;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BlueSkyPrkBridgeDep extends MainAutonomous {
    private String className = this.getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences();

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
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
         */
        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Transition
            encoderDrive("left", minPower, 0.25);
            encoderDrive("front", minPower, 0.25);
            // Recognize skystone
            recognizeTarget("Stone Target");
            switch (skystonePosition) {
                case "left":
                    // 1st skystone
                    encoderDriveDist("left", minPower, travelX);
                    grabSkystone(minPower);
                    depotToBuildingSite("blue", minPower, minTurnPower, 3);
                    encoderDriveDist("left", minPower, (-travelX + firstPlacement));
                    buildSkystone("blue", minPower, 1);
                    // 2nd skystone
                    buildingSiteToDepot("blue", minPower, minTurnPower, 4);
                    encoderDriveDist("right", minPower, (-travelX + firstPlacement));
                    grabSkystone(minPower);
                    depotToBuildingSite("blue", minPower, minTurnPower, 4);
                    encoderDriveDist("left", minPower, (-travelX + secondPlacement));
                    buildSkystone("blue", minPower, 1);
                    encoderDriveDist("right", minPower, (-centerPlacement + secondPlacement));

                case "center":
                    encoderDriveDist("right", minPower, travelX);
                    grabSkystone(minPower);
                    depotToBuildingSite("blue", minPower, minTurnPower, 3);
                    encoderDriveDist("left", minPower, (travelX + 18));
                    buildSkystone("blue", minPower, minTurnPower, 1);
                    buildingSiteToDepot("blue", minPower, minTurnPower, 4.75);
                    encoderDriveDist("right", minPower, travelX);

                case "right":
                    encoderDriveDist("right", minPower, (fullSkystoneDist + halfSkystoneDist));
                    grabSkystone(minPower);
                    depotToBuildingSite("blue", minPower, minTurnPower, 3);
                    encoderDriveDist("left", minPower, (fullSkystoneDist + halfSkystoneDist + 18));
                    buildSkystone("blue", minPower, minTurnPower, 1);
            }


            encoderDrive("right", minPower, 1);
            // Grab 2nd skystone
            switch (skystonePosition) {
                case "left":
                    encoderDrive("left", minPower, halfSkystoneDist);
                    grabSkystone(minPower);
                    encoderDrive("right", minPower, halfSkystoneDist);
                case "center":
                    encoderDrive("right", minPower, halfSkystoneDist);
                    grabSkystone(minPower);
                    encoderDrive("left", minPower, halfSkystoneDist);
                case "right":
                    gyroTurn(90, minTurnPower);
                    encoderDrive("front", minPower, 0.25);
                    encoderDrive("left", minPower, 0.25);
                    captureLeftSkystone();
                    encoderDrive("right", minPower, 0.25);
                    encoderDrive("back", minPower, 0.25);
                    gyroTurn(-90, minTurnPower);
            }
            // Deliver 2nd skystone
            encoderDrive("left", minPower, 1);
            depotToBuildingsite("blue", minPower, minTurnPower);
            buildSkystone("blue", minPower, minTurnPower, 2);
            // Parking
            encoderDrive("right", minPower, 1.625);
        }

        resetMotor();
        visionTargets.deactivate();
    }
}
